/**
 * @file PowerToggle.cpp
 *
 * @brief Debounced touch button: taps toggle power, a long hold toggles red alert.
 *
 * Owns the touch pin and the model's off / on / red-alert state. Each accepted
 * touch flips that state and latches a one-shot transition for a dispatcher task
 * to pick up via power_toggle_consume_transition() -- this module has no
 * knowledge of what should happen on a transition, only that one occurred.
 *
 * A touch is classified by how long it is held:
 *   - released inside CAL_TOUCH_HOLD_MS (a tap)  -> off <-> on, and red alert -> off
 *   - still held at CAL_TOUCH_HOLD_MS (a hold)   -> off/on -> red alert,
 *                                                   red alert -> on
 * A hold acts the moment the threshold is crossed, not on release, so the model
 * reacts while the finger is still down; the release that follows is then
 * swallowed rather than also counting as a tap. A tap, by contrast, can only be
 * recognised once the finger lifts -- that is the cost of telling the two apart,
 * and it is why the action is applied on the falling edge rather than the rising
 * one as it was before red alert existed.
 *
 * The button sits on D12, which has no hardware interrupt (only D2/D3 do on the
 * Nano), so the edges are found by polling instead of in an ISR.
 * power_toggle_update() therefore has to run on every scheduler pass: a touch
 * shorter than one pass would otherwise be missed entirely.
 *
 * Both edges are debounced by one shared settle window: an edge inside
 * CAL_TOUCH_DEBOUNCE_MS of the previous accepted one is left unread, so contact
 * bounce neither registers as a touch nor cuts a hold short.
 *
 * State after a reset is off, so the first touch is always a turn-on (or, held,
 * a jump straight to red alert). No transition is latched at init -- nothing
 * chirps or lights on power-up, and a pin that is already active at boot is not
 * read as a touch.
 */

/* File header */
#include "PowerToggle.h"

/* System headers */
#include <Arduino.h>

/* Third-party header files */
/* None */

/* Project headers */
#include "Calibration.h"

/* Constants, macros, datatypes */
typedef enum
{
    POWER_STATE_OFF = 0,
    POWER_STATE_ON,
    POWER_STATE_RED_ALERT
} power_state_t;

/* Static variable definitions */
static bool s_last_level_active     = false;
static unsigned long s_last_edge_ms = 0;

/* A press that passed debounce and has not been released or acted on yet. */
static bool s_touch_down              = false;
static bool s_hold_applied            = false;
static unsigned long s_touch_start_ms = 0;

/* A reset leaves the model powered down -- the first touch is what turns it on. */
static power_state_t s_state                          = POWER_STATE_OFF;
static power_toggle_transition_t s_pending_transition = POWER_TOGGLE_NONE;

/* Static function prototypes */
static void power_toggle_apply_tap(void);
static void power_toggle_apply_hold(void);
static void power_toggle_enter(power_state_t state, power_toggle_transition_t transition);

/**
 * @brief Configure the touch pin and seed the edge detector with its resting
 *        level, so a pin that is already active at boot is not read as a touch.
 */
void power_toggle_init(void)
{
    pinMode(CAL_PIN_TOUCH_BUTTON, CAL_TOUCH_PIN_MODE);
    s_last_level_active = (digitalRead(CAL_PIN_TOUCH_BUTTON) == CAL_TOUCH_ACTIVE_LEVEL);
    s_last_edge_ms      = millis();
}

/**
 * @brief Poll the touch pin, classify the touch as tap or hold and apply it.
 *        Must be polled every scheduler pass so no touch is missed and so the
 *        transition reaches the dispatcher with minimal latency.
 */
void power_toggle_update(void)
{
    bool level_active    = (digitalRead(CAL_PIN_TOUCH_BUTTON) == CAL_TOUCH_ACTIVE_LEVEL);
    unsigned long now_ms = millis();

    if (level_active != s_last_level_active)
    {
        if (now_ms - s_last_edge_ms <= CAL_TOUCH_DEBOUNCE_MS)
        {
            /* Contact bounce. Leave the edge unread: if the new level is real it
             * is still there once the settle window has passed. */
            return;
        }

        s_last_edge_ms      = now_ms;
        s_last_level_active = level_active;

        if (level_active)
        {
            s_touch_down     = true;
            s_hold_applied   = false;
            s_touch_start_ms = now_ms;
        }
        else
        {
            if (s_touch_down && !s_hold_applied)
            {
                power_toggle_apply_tap();
            }
            s_touch_down = false;
        }
        return;
    }

    /* Level unchanged: the only thing left to check is a press that has just
     * grown long enough to count as a hold. */
    if (s_touch_down && !s_hold_applied && (now_ms - s_touch_start_ms >= CAL_TOUCH_HOLD_MS))
    {
        s_hold_applied = true;
        power_toggle_apply_hold();
    }
}

/**
 * @brief True whenever the model is lit -- normal on or red alert alike.
 */
bool power_toggle_is_on(void) { return (s_state != POWER_STATE_OFF); }

/**
 * @brief True only in red alert.
 */
bool power_toggle_is_red_alert(void) { return (s_state == POWER_STATE_RED_ALERT); }

/**
 * @brief Take and clear the pending transition, if any. Returns
 *        POWER_TOGGLE_NONE when nothing has changed since the last call.
 */
power_toggle_transition_t power_toggle_consume_transition(void)
{
    power_toggle_transition_t transition = s_pending_transition;
    s_pending_transition                 = POWER_TOGGLE_NONE;
    return transition;
}

/**
 * @brief Apply a short touch: toggle power. Red alert counts as on, so a tap
 *        out of it powers the model down rather than stepping back to normal.
 */
static void power_toggle_apply_tap(void)
{
    if (s_state == POWER_STATE_OFF)
    {
        power_toggle_enter(POWER_STATE_ON, POWER_TOGGLE_TURNED_ON);
    }
    else
    {
        power_toggle_enter(POWER_STATE_OFF, POWER_TOGGLE_TURNED_OFF);
    }
}

/**
 * @brief Apply a long touch: toggle red alert. Holding from off brings the model
 *        up straight into red alert; holding while in it drops back to normal on.
 */
static void power_toggle_apply_hold(void)
{
    if (s_state == POWER_STATE_RED_ALERT)
    {
        power_toggle_enter(POWER_STATE_ON, POWER_TOGGLE_TURNED_ON);
    }
    else
    {
        power_toggle_enter(POWER_STATE_RED_ALERT, POWER_TOGGLE_TURNED_RED_ALERT);
    }
}

/**
 * @brief Move to @p state and latch @p transition for the dispatcher. A
 *        transition not yet consumed is overwritten -- the latest touch wins.
 */
static void power_toggle_enter(power_state_t state, power_toggle_transition_t transition)
{
    s_state              = state;
    s_pending_transition = transition;
}
