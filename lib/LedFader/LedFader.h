/**
 * @file LedFader.h
 *
 * @brief Header file for LedFader.cpp
 */

#ifndef LED_FADER_H
#define LED_FADER_H

/* System headers */
#include <stdbool.h>

/* Third-party header files */
/* None */

/* Project headers */
/* None */

/* Constants, Datatypes */
/* Which channels exist, and every fade parameter, live in Calibration.h.
 * led_fader_update() expects to be scheduled at CAL_LED_TICK_MS. */

/**
 * Which set of channels is lit. The channel masks these names select between
 * live in Calibration.h; a channel outside the current scene's mask ramps down
 * to off rather than cutting out.
 */
typedef enum
{
    LED_FADER_SCENE_OFF = 0,
    LED_FADER_SCENE_NORMAL,
    LED_FADER_SCENE_RED_ALERT
} led_fader_scene_t;

/* Function prototypes */
void led_fader_init(void);
void led_fader_update(void);
void led_fader_set_scene(led_fader_scene_t scene);

#endif /* LED_FADER_H */
