/**
 * @file main.cpp
 *
 * @brief Entry point. Wires up the cooperative scheduler and registers
 *        each module's update task.
 */

/* File header */
/* None */

/* System headers */
#include <Arduino.h>

/* Third-party header files */
/* None */

/* Project headers */
#include "Calibration.h"
#include "ChirpNotifier.h"
#include "LedFader.h"
#include "PowerToggle.h"
#include "Scheduler.h"

/* Constants, macros, datatypes */
/* None */

/* Static variable definitions */
/* None */

/* Static function prototypes */
static void dispatch_power_toggle(void);

/**
 * @brief Initialise modules and register their tasks with the scheduler.
 */
void setup(void)
{
    power_toggle_init();
    chirp_notifier_init();
    led_fader_init();

    scheduler_init();
    /* Both need polling on every pass: PowerToggle because the touch pin has
     * no interrupt and a short touch between passes would be lost,
     * ChirpNotifier for its sub-30ms note timing. */
    scheduler_add_task(power_toggle_update, 0);
    scheduler_add_task(dispatch_power_toggle, 0);
    scheduler_add_task(chirp_notifier_update, 0);
    scheduler_add_task(led_fader_update, CAL_LED_TICK_MS);
}

/**
 * @brief Hand control to the scheduler. No module here may block, or it
 *        will stall every other task's timing.
 */
void loop(void)
{
    /* Run the scheduler */
    scheduler_run();
}

/**
 * @brief Turn a pending PowerToggle transition into module-level actions: pick
 *        the LED scene the new state calls for and sound its chirp.
 *        This is the one place allowed to reach across modules -- neither
 *        PowerToggle, ChirpNotifier nor LedFader know about each other.
 */
static void dispatch_power_toggle(void)
{
    power_toggle_transition_t transition = power_toggle_consume_transition();

    switch (transition)
    {
        case POWER_TOGGLE_TURNED_ON:
            led_fader_set_scene(LED_FADER_SCENE_NORMAL);
            chirp_notifier_play(CHIRP_SEQUENCE_ON);
            break;

        case POWER_TOGGLE_TURNED_RED_ALERT:
            led_fader_set_scene(LED_FADER_SCENE_RED_ALERT);
            /* No red-alert chirp of its own yet -- acknowledge the hold with the
             * power-on sequence so the 3 s threshold is audibly confirmed. */
            chirp_notifier_play(CHIRP_SEQUENCE_ON);
            break;

        case POWER_TOGGLE_TURNED_OFF:
            led_fader_set_scene(LED_FADER_SCENE_OFF);
            chirp_notifier_play(CHIRP_SEQUENCE_OFF);
            break;

        case POWER_TOGGLE_NONE:
        default:
            break;
    }
}
