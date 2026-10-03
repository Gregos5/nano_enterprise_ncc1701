/**
 * @file LedFader.cpp
 *
 * @brief Breathing-LED effect across every LED group on the model.
 *
 * One led_fader_update() call advances every channel by one step and does no
 * waiting of its own -- the caller (the cooperative scheduler, at
 * CAL_LED_TICK_MS) supplies the pacing that used to come from a
 * blocking delay(). Each channel has its own period, min/max brightness and
 * top/bottom pauses, all read from Calibration.h at init.
 *
 * Fade rate is not constant: it scales linearly with the current PWM level,
 * from CAL_LED_MIN_RATE_GAIN_PERCENT of full rate at 0 PWM up to full rate at
 * 255, which tracks how the eye perceives brightness far better than a linear
 * ramp does. Because the rate varies along the ramp, the per-tick step that
 * makes a ramp finish in the requested time is not (range / ticks) -- it comes
 * from integrating dt = dB / (rate * gain(B)) over the ramp, which for a gain
 * that is linear in B gives a logarithm. That integral is evaluated once per
 * channel at init (see led_fader_ramp_step_q16), never in the update path;
 * stepping itself is integer-only Q16 fixed point.
 *
 * Stepping evaluates the gain at the brightness the tick *starts* from, which
 * is a forward-Euler step and so runs a few percent long (measured: 5.03s for
 * the 5s wings, 10.22s for the 10s bridge). That is a known trade, not a bug --
 * one multiply per channel per tick is worth more here than exact periods.
 *
 * What is lit is chosen by scene, not by a global on/off: led_fader_set_scene()
 * swaps in that scene's channel mask (from Calibration.h). A channel the new
 * scene does not include is not cut instantly -- it ramps down to 0 on the same
 * rate curve and then holds off; a channel the new scene adds starts climbing
 * from wherever its brightness happens to be, rather than restarting its cycle.
 * LED_FADER_SCENE_OFF is simply the scene whose mask is empty, so powering down
 * needs no separate code path.
 *
 * A channel calibrated with min == max (the saucer) is a "hold" channel: it has
 * no breathing span, so it fades in to that level and stays there, and fades
 * back out on power off.
 *
 * The module comes out of led_fader_init() in LED_FADER_SCENE_OFF with every
 * channel dark, so a reset leaves the model unlit until the first touch. Because
 * every channel also starts at brightness 0, that first fade-in looks identical
 * to any later toggle back on.
 */

/* File header */
#include "LedFader.h"

/* System headers */
#include <Arduino.h>
#include <math.h>

/* Third-party header files */
/* None */

/* Project headers */
#include "Calibration.h"

/* Constants, macros, datatypes */
/* Brightness is carried as Q16 fixed point (PWM counts << 16) so that a ramp
 * lasting thousands of ticks can advance by a fraction of a count per tick. */
#define LED_FADER_Q16_SHIFT     16
#define LED_FADER_LEVEL_MAX_Q16 (((uint32_t) CAL_LED_PWM_MAX) << LED_FADER_Q16_SHIFT)

/* Rate gain is Q8: 256 == full rate. Capping a step at full scale keeps
 * (step_q16 * gain_q8) inside 32 bits. */
#define LED_FADER_GAIN_Q8_UNITY 256U
#define LED_FADER_GAIN_Q8_MIN   ((CAL_LED_MIN_RATE_GAIN_PERCENT * LED_FADER_GAIN_Q8_UNITY) / 100U)

typedef enum
{
    LED_FADER_PHASE_RISING = 0,
    LED_FADER_PHASE_PAUSE_TOP,
    LED_FADER_PHASE_FALLING,
    LED_FADER_PHASE_PAUSE_BOTTOM
} led_fader_phase_t;

typedef struct
{
    uint32_t level_q16;        /* current brightness */
    uint32_t min_level_q16;    /* bottom of the breathing range */
    uint32_t max_level_q16;    /* top of the breathing range */
    uint32_t breathe_step_q16; /* per tick at full gain, min <-> max; 0 if min == max */
    uint32_t full_step_q16;    /* per tick at full gain, 0 <-> max */
    unsigned long pause_start_ms;
    led_fader_phase_t phase;
    uint8_t last_pwm;
    bool pwm_written;
    bool flickering;           /* red alert saucer: base level reached, now flickering */
    bool flicker_on;           /* current flicker state */
    unsigned long next_flicker_ms;
} led_fader_state_t;

/* Static variable definitions */
static led_fader_state_t s_channels[CAL_LED_CHANNEL_COUNT];
/* Bit per channel: set means "breathe", clear means "ramp down and stay off". */
static uint8_t s_scene_mask = 0U;
static led_fader_scene_t s_scene = LED_FADER_SCENE_OFF;
/* Set once every channel outside the mask has reached 0, so a fully dark model
 * costs nothing per tick. Cleared whenever the scene changes. */
static bool s_settled = true;

/* Static function prototypes */
static uint8_t led_fader_scene_mask(led_fader_scene_t scene);
static void led_fader_calibrate_channel(uint8_t index);
static uint32_t led_fader_ramp_step_q16(uint16_t from_pwm, uint16_t to_pwm, uint16_t ramp_ms);
static uint32_t led_fader_increment_q16(uint32_t step_q16, uint32_t level_q16);
static uint32_t led_fader_rise_step_q16(const led_fader_state_t *state);
static void led_fader_advance_breathing(uint8_t index, unsigned long now_ms);
static bool led_fader_is_hold(uint8_t index);
static uint32_t led_fader_target_q16(uint8_t index);
static uint32_t led_fader_flicker_low_q16(void);
static bool led_fader_approach_target(led_fader_state_t *state, uint32_t target_q16);
static void led_fader_advance_flicker(uint8_t index, unsigned long now_ms);
static bool led_fader_flickers(uint8_t index);
static bool led_fader_advance_off(led_fader_state_t *state);
static void led_fader_write(led_fader_state_t *state, uint8_t pin);

/**
 * @brief Configure every LED pin, drive it low and pre-compute its ramp rates.
 *
 * Leaves the module switched off: a reset brings every LED up dark, and nothing
 * lights until led_fader_set_enabled(true) arrives from the first touch.
 */
void led_fader_init(void)
{
    uint8_t index;

    for (index = 0; index < CAL_LED_CHANNEL_COUNT; index++)
    {
        pinMode(s_cal_led_channels[index].pin, OUTPUT);
        /* Don't rely on the port register's reset value -- drive it low. */
        analogWrite(s_cal_led_channels[index].pin, 0);
        led_fader_calibrate_channel(index);
    }

    /* Unconnected A0 is noisy enough to give each boot a different flicker. */
    randomSeed(analogRead(A0));

    s_scene      = LED_FADER_SCENE_OFF;
    s_scene_mask = led_fader_scene_mask(LED_FADER_SCENE_OFF);
    s_settled    = true;
}

/**
 * @brief Advance every channel by one step -- through its breathing cycle if the
 *        current scene includes it, towards off if it does not -- then push the
 *        new brightness out. Idle (no analogWrite) once nothing is left to move.
 */
void led_fader_update(void)
{
    unsigned long now_ms = millis();
    bool work_remains    = false;
    uint8_t index;

    if (s_settled)
    {
        /* Nothing to do; every channel this scene excludes is already at 0 and
         * the scene itself lights nothing. */
        return;
    }

    for (index = 0; index < CAL_LED_CHANNEL_COUNT; index++)
    {
        if ((s_scene_mask & CAL_LED_CHANNEL_BIT(index)) != 0U)
        {
            if (led_fader_flickers(index))
            {
                led_fader_advance_flicker(index, now_ms);
                work_remains = true;
            }
            else if (led_fader_is_hold(index))
            {
                work_remains |= !led_fader_approach_target(&s_channels[index],
                                                           led_fader_target_q16(index));
            }
            else
            {
                led_fader_advance_breathing(index, now_ms);
                work_remains = true;
            }
        }
        else if (!led_fader_advance_off(&s_channels[index]))
        {
            work_remains = true;
        }

        led_fader_write(&s_channels[index], s_cal_led_channels[index].pin);
    }

    s_settled = !work_remains;
}

/**
 * @brief Switch to a scene. Channels the scene drops ramp down to 0 rather than
 *        cutting out; channels it adds climb from their current brightness.
 */
void led_fader_set_scene(led_fader_scene_t scene)
{
    uint8_t mask = led_fader_scene_mask(scene);
    uint8_t index;

    if (mask == s_scene_mask)
    {
        return;
    }

    for (index = 0; index < CAL_LED_CHANNEL_COUNT; index++)
    {
        bool was_lit = ((s_scene_mask & CAL_LED_CHANNEL_BIT(index)) != 0U);
        bool is_lit  = ((mask & CAL_LED_CHANNEL_BIT(index)) != 0U);

        if (is_lit && !was_lit)
        {
            /* Climb to the top from wherever this channel was left. */
            s_channels[index].phase = LED_FADER_PHASE_RISING;
        }

        /* A flicker belongs to the scene that started it. */
        s_channels[index].flickering = false;
    }

    s_scene      = scene;
    s_scene_mask = mask;
    s_settled    = false;
}

/**
 * @brief Map a scene onto its channel mask in Calibration.h. A switch rather
 *        than an array index so the two enums need not share an order.
 */
static uint8_t led_fader_scene_mask(led_fader_scene_t scene)
{
    switch (scene)
    {
        case LED_FADER_SCENE_NORMAL:
            return s_cal_led_scene_masks[CAL_LED_SCENE_NORMAL];

        case LED_FADER_SCENE_RED_ALERT:
            return s_cal_led_scene_masks[CAL_LED_SCENE_RED_ALERT];

        case LED_FADER_SCENE_OFF:
        default:
            return s_cal_led_scene_masks[CAL_LED_SCENE_OFF];
    }
}

/**
 * @brief Turn one channel's calibration entry into the levels and per-tick
 *        steps the update path uses. Called once, from led_fader_init().
 */
static void led_fader_calibrate_channel(uint8_t index)
{
    const cal_led_channel_t *cal = &s_cal_led_channels[index];
    led_fader_state_t *state     = &s_channels[index];

    uint16_t min_pwm = (uint16_t) ((cal->min_percent * (uint16_t) CAL_LED_PWM_MAX) / 100U);
    uint16_t max_pwm = (uint16_t) ((cal->max_percent * (uint16_t) CAL_LED_PWM_MAX) / 100U);
    uint16_t pause_ms;
    uint16_t ramp_ms;

    if (max_pwm < min_pwm)
    {
        max_pwm = min_pwm; /* mis-calibrated: treat as "hold at min" */
    }

    /* period_ms covers the whole cycle, pauses included, so what is left over
     * is split between the two ramps. */
    pause_ms = cal->pause_bottom_ms + cal->pause_top_ms;
    ramp_ms  = (cal->period_ms > pause_ms) ? ((cal->period_ms - pause_ms) / 2U) : 0U;
    if (ramp_ms < CAL_LED_TICK_MS)
    {
        ramp_ms = CAL_LED_TICK_MS; /* floor of one tick per ramp */
    }

    state->min_level_q16 = ((uint32_t) min_pwm) << LED_FADER_Q16_SHIFT;
    state->max_level_q16 = ((uint32_t) max_pwm) << LED_FADER_Q16_SHIFT;
    /* Both breathing directions cover the same brightness span on the same gain
     * curve, so one step serves for rising and falling alike. This is 0 on a
     * hold channel (min == max), which is what led_fader_rise_step_q16() looks
     * for. */
    state->breathe_step_q16 = led_fader_ramp_step_q16(min_pwm, max_pwm, ramp_ms);
    /* Covers the whole 0..max span: used to ramp down to off, and to fade in
     * from off on a channel that has no breathing span of its own. */
    state->full_step_q16 = led_fader_ramp_step_q16(0, max_pwm, ramp_ms);

    /* Start dark and let the rising phase fade in, so a channel lights the same
     * way on the first touch after a reset as it does on any later toggle back
     * on, rather than snapping to a level. */
    state->level_q16      = 0U;
    state->phase          = LED_FADER_PHASE_RISING;
    state->pause_start_ms = 0;
    state->last_pwm       = 0;
    state->pwm_written    = true; /* led_fader_init() has already driven it low */
}

/**
 * @brief Per-tick step for the rising phase.
 *
 * A hold channel (min == max, e.g. the always-on saucer) has no breathing span
 * and so no breathing step. It still has to be able to climb -- from 0 at power
 * on, and from wherever a ramp-down to off left it -- so it rises at the
 * full-span rate instead. Without this a hold channel would sit at 0 forever
 * once switched off, since a zero step can never reach its target.
 */
static uint32_t led_fader_rise_step_q16(const led_fader_state_t *state)
{
    return (state->breathe_step_q16 != 0U) ? state->breathe_step_q16 : state->full_step_q16;
}

/**
 * @brief Per-tick step, at full gain, that walks from @p from_pwm to
 *        @p to_pwm in @p ramp_ms given the linear rate-gain curve.
 *
 * With gain(B) = g0 + k*B, the time to cross the ramp is
 * integral(dB / (rate * gain(B))) = ln((g0 + k*to) / (g0 + k*from)) / (k*rate),
 * so the full-gain rate that lands on @p ramp_ms is that logarithm over the
 * tick count. Float is fine here: this runs once per channel at startup, not
 * in the update path.
 *
 * @return Q16 PWM counts per tick; 0 only when the ramp has no span.
 */
static uint32_t led_fader_ramp_step_q16(uint16_t from_pwm, uint16_t to_pwm, uint16_t ramp_ms)
{
    float min_gain = (float) CAL_LED_MIN_RATE_GAIN_PERCENT / 100.0F;
    float slope    = (1.0F - min_gain) / (float) CAL_LED_PWM_MAX;
    float ticks    = (float) ramp_ms / (float) CAL_LED_TICK_MS;
    float span;
    float step_q16;

    if (to_pwm <= from_pwm)
    {
        return 0U; /* nothing to traverse -- the channel holds still */
    }

    if (slope < 1.0e-6F)
    {
        /* Gain pinned at 100%: a plain constant-rate ramp. */
        span = (float) (to_pwm - from_pwm);
    }
    else
    {
        span = log((min_gain + slope * (float) to_pwm) / (min_gain + slope * (float) from_pwm)) /
               slope;
    }

    step_q16 = (span / ticks) * (float) (1UL << LED_FADER_Q16_SHIFT) + 0.5F;

    if (step_q16 < 1.0F)
    {
        return 1U; /* never stall a ramp that has span left to cover */
    }
    if (step_q16 > (float) LED_FADER_LEVEL_MAX_Q16)
    {
        return LED_FADER_LEVEL_MAX_Q16; /* saturate: full scale in one tick */
    }
    return (uint32_t) step_q16;
}

/**
 * @brief Scale a full-gain step by the rate gain at the current brightness:
 *        full rate at full-scale PWM down to CAL_LED_MIN_RATE_GAIN_PERCENT at 0.
 */
static uint32_t led_fader_increment_q16(uint32_t step_q16, uint32_t level_q16)
{
    uint16_t pwm = (uint16_t) (level_q16 >> LED_FADER_Q16_SHIFT);
    uint32_t gain_q8;
    uint32_t increment_q16;

    gain_q8 = LED_FADER_GAIN_Q8_MIN +
              (((LED_FADER_GAIN_Q8_UNITY - LED_FADER_GAIN_Q8_MIN) * (uint32_t) pwm) /
               (uint32_t) CAL_LED_PWM_MAX);

    increment_q16 = (step_q16 * gain_q8) >> 8;

    if ((increment_q16 == 0U) && (step_q16 != 0U))
    {
        increment_q16 = 1U; /* keep a slow ramp creeping rather than stalling */
    }
    return increment_q16;
}

/**
 * @brief Advance one channel through its rise / pause-at-top / fall /
 *        pause-at-bottom cycle.
 */
static void led_fader_advance_breathing(uint8_t index, unsigned long now_ms)
{
    const cal_led_channel_t *cal = &s_cal_led_channels[index];
    led_fader_state_t *state     = &s_channels[index];
    uint32_t increment_q16;

    switch (state->phase)
    {
        case LED_FADER_PHASE_RISING:
            increment_q16 =
                led_fader_increment_q16(led_fader_rise_step_q16(state), state->level_q16);
            if (state->level_q16 + increment_q16 >= state->max_level_q16)
            {
                state->level_q16      = state->max_level_q16;
                state->phase          = LED_FADER_PHASE_PAUSE_TOP;
                state->pause_start_ms = now_ms;
            }
            else
            {
                state->level_q16 += increment_q16;
            }
            break;

        case LED_FADER_PHASE_PAUSE_TOP:
            if (now_ms - state->pause_start_ms >= cal->pause_top_ms)
            {
                state->phase = LED_FADER_PHASE_FALLING;
            }
            break;

        case LED_FADER_PHASE_FALLING:
            increment_q16 = led_fader_increment_q16(state->breathe_step_q16, state->level_q16);
            if (state->level_q16 <= state->min_level_q16 + increment_q16)
            {
                state->level_q16      = state->min_level_q16;
                state->phase          = LED_FADER_PHASE_PAUSE_BOTTOM;
                state->pause_start_ms = now_ms;
            }
            else
            {
                state->level_q16 -= increment_q16;
            }
            break;

        case LED_FADER_PHASE_PAUSE_BOTTOM:
            if (now_ms - state->pause_start_ms >= cal->pause_bottom_ms)
            {
                state->phase = LED_FADER_PHASE_RISING;
            }
            break;
    }
}

/**
 * @brief True for a channel with no breathing span (min == max): it holds one
 *        level, which the active scene may override.
 */
static bool led_fader_is_hold(uint8_t index)
{
    return s_cal_led_channels[index].min_percent == s_cal_led_channels[index].max_percent;
}

/**
 * @brief The level a channel should settle at in the current scene, as Q16.
 *        Red alert overrides the wings and saucer; everything else uses its
 *        calibrated maximum.
 */
static uint32_t led_fader_target_q16(uint8_t index)
{
    uint8_t percent = s_cal_led_channels[index].max_percent;

    if (led_fader_flickers(index))
    {
        percent = CAL_RED_ALERT_FLICKER_HIGH_PERCENT;
    }

    return ((uint32_t) ((percent * CAL_LED_PWM_MAX) / 100U)) << LED_FADER_Q16_SHIFT;
}

/**
 * @brief The low flicker level as Q16, so a flicker never drops to fully off.
 */
static uint32_t led_fader_flicker_low_q16(void)
{
    return ((uint32_t) ((CAL_RED_ALERT_FLICKER_LOW_PERCENT * CAL_LED_PWM_MAX) / 100U))
           << LED_FADER_Q16_SHIFT;
}

/**
 * @brief Move one channel towards @p target_q16 at the rate its ramp allows,
 *        rising or falling as needed.
 *
 * @return true if the channel is already at the target (nothing moved).
 */
static bool led_fader_approach_target(led_fader_state_t *state, uint32_t target_q16)
{
    uint32_t increment_q16;

    if (state->level_q16 < target_q16)
    {
        increment_q16 = led_fader_increment_q16(led_fader_rise_step_q16(state), state->level_q16);
        state->level_q16 = ((state->level_q16 + increment_q16) >= target_q16)
                               ? target_q16
                               : (state->level_q16 + increment_q16);
    }
    else if (state->level_q16 > target_q16)
    {
        increment_q16 = led_fader_increment_q16(state->full_step_q16, state->level_q16);
        state->level_q16 = ((state->level_q16 - target_q16) <= increment_q16)
                               ? target_q16
                               : (state->level_q16 - increment_q16);
    }

    return (state->level_q16 == target_q16);
}

/**
 * @brief True for a channel that flickers in the current scene: the wings and
 *        saucer during red alert.
 */
static bool led_fader_flickers(uint8_t index)
{
    return (s_scene == LED_FADER_SCENE_RED_ALERT) &&
           ((index == CAL_LED_CHANNEL_WINGS) || (index == CAL_LED_CHANNEL_SAUCER));
}

/**
 * @brief Red alert wings and saucer: fade up to the high level, then flicker
 *        between the high and low levels at random intervals.
 */
static void led_fader_advance_flicker(uint8_t index, unsigned long now_ms)
{
    led_fader_state_t *state = &s_channels[index];
    uint32_t target_q16      = led_fader_target_q16(index);

    if (!state->flickering)
    {
        if (led_fader_approach_target(state, target_q16))
        {
            state->flickering      = true;
            state->flicker_on      = true;
            state->next_flicker_ms = now_ms;
        }
        return;
    }

    if (now_ms >= state->next_flicker_ms)
    {
        state->flicker_on      = !state->flicker_on;
        state->next_flicker_ms = now_ms + (unsigned long) random(
                                              CAL_RED_ALERT_SAUCER_FLICKER_MIN_MS,
                                              CAL_RED_ALERT_SAUCER_FLICKER_MAX_MS + 1);
    }

    state->level_q16 = state->flicker_on ? target_q16 : led_fader_flicker_low_q16();
}

/**
 * @brief Ramp one channel down towards off.
 *
 * @return true once the channel has reached 0.
 */
static bool led_fader_advance_off(led_fader_state_t *state)
{
    uint32_t increment_q16 = led_fader_increment_q16(state->full_step_q16, state->level_q16);

    if (state->level_q16 <= increment_q16)
    {
        state->level_q16 = 0U;
        return true;
    }

    state->level_q16 -= increment_q16;
    return false;
}

/**
 * @brief Push a channel's brightness to its pin, skipping the write when the
 *        PWM count has not actually changed (pauses, and held channels).
 */
static void led_fader_write(led_fader_state_t *state, uint8_t pin)
{
    uint8_t pwm = (uint8_t) (state->level_q16 >> LED_FADER_Q16_SHIFT);

    if (!state->pwm_written || (pwm != state->last_pwm))
    {
        analogWrite(pin, pwm);
        state->last_pwm    = pwm;
        state->pwm_written = true;
    }
}
