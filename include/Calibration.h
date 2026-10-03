/**
 * @file Calibration.h
 *
 * @brief Single place to tune the build: pin assignments, chirp note tables
 *        and LED fading parameters.
 *
 * Nothing here is logic -- only the numbers the modules read at init time, and
 * the shape of the tables that hold them. A module owns *how* it behaves; this
 * file owns *with what values*. Change a pin or a fade curve here, never
 * inside a module.
 *
 * This header deliberately includes no project headers, so a module pulling it
 * in never ends up depending on another module. The tables are file-scope
 * statics rather than externs for the same reason: each is pulled in only by
 * the module that uses it, and unused copies are dropped at link time
 * (-ffunction-sections/-fdata-sections + --gc-sections).
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

/* System headers */
#include <stdint.h>

/* Third-party header files */
/* None */

/* Project headers */
/* None -- see the file brief. */

/* Constants, macros, datatypes */

/* -- GPIO ---------------------------------------------------------------- */
/* Buzzer sits on D11 (Timer2). tone() also drives Timer2, so D3's PWM is
 * unavailable -- nothing uses it. The four LED pins split across Timer0
 * (D5/D6) and Timer1 (D9/D10); all four are PWM-capable. */
#define CAL_PIN_BUZZER        ((uint8_t) 11)
#define CAL_PIN_LED_WINGS     ((uint8_t) 5)
#define CAL_PIN_LED_RED_ALERT ((uint8_t) 6)
#define CAL_PIN_LED_SAUCER    ((uint8_t) 9)
#define CAL_PIN_LED_BRIDGE    ((uint8_t) 10)
/* D12 has no hardware interrupt (only D2/D3 do on the Nano), so PowerToggle
 * polls it instead of using an ISR -- see PowerToggle.cpp. */
#define CAL_PIN_TOUCH_BUTTON ((uint8_t) 12)

/* -- Touch button -------------------------------------------------------- */
/* Ignore re-triggers within this window after an accepted touch. */
#define CAL_TOUCH_DEBOUNCE_MS ((uint16_t) 100)
/* A touch held at least this long is a hold, not a tap: it toggles red alert
 * instead of toggling power. A touch released sooner is a tap. */
#define CAL_TOUCH_HOLD_MS ((uint16_t) 3000)
/* Touch module idles low and pulses high while touched. HIGH/LOW and INPUT
 * come from Arduino.h, which every including .cpp pulls in ahead of this. */
#define CAL_TOUCH_ACTIVE_LEVEL HIGH
#define CAL_TOUCH_PIN_MODE     INPUT /* INPUT_PULLUP if the source floats */

/* -- Chirp tables -------------------------------------------------------- */
/**
 * One note of a chirp sequence: a tone of @c freq_hz held for @c duration_ms,
 * followed by @c gap_ms of silence before the next note.
 */
typedef struct
{
    uint16_t freq_hz;
    uint16_t duration_ms;
    uint16_t gap_ms;
} cal_chirp_note_t;

/* 4-note power-on chirp: low -> high -> high-mid -> low. Frequencies and
 * durations were extracted from tng_chirp3_clean.mp3 by STFT peak-frequency
 * analysis. */
/* clang-format off */
static const cal_chirp_note_t s_cal_chirp_sequence_on[] = {
    /* freq_hz  duration_ms  gap_ms */
    {    3600,          70,     20 },
    {    4650,          30,     10 },
    {    4200,          40,     50 },
    {    3800,          35,      0 }
};
/* clang-format on */
#define CAL_CHIRP_SEQUENCE_ON_LEN                                                                  \
    (sizeof(s_cal_chirp_sequence_on) / sizeof(s_cal_chirp_sequence_on[0]))

/* Placeholder 2-note "power down" chirp -- tune to taste. */
/* clang-format off */
static const cal_chirp_note_t s_cal_chirp_sequence_off[] = {
    /* freq_hz  duration_ms  gap_ms */
    {    4200,          50,     30 },
    {    2600,          90,      0 }
};
/* clang-format on */
#define CAL_CHIRP_SEQUENCE_OFF_LEN                                                                 \
    (sizeof(s_cal_chirp_sequence_off) / sizeof(s_cal_chirp_sequence_off[0]))

/* Red alert entry: a stuttering "communication error" chirp -- the tone jumps
 * back and forth between high and low and sags away at the end, like a link
 * failing mid-transmission. Hand-tuned; not extracted from a recording. */
/* clang-format off */
static const cal_chirp_note_t s_cal_chirp_sequence_red_alert[] = {
    /* freq_hz  duration_ms  gap_ms */
    {    2400,          40,     15 },
    {    1300,          40,     15 },
    {    2800,          30,     15 },
    {    1000,          35,     20 },
    {    2100,          30,     15 },
    {     700,          90,      0 }
};
/* clang-format on */
#define CAL_CHIRP_SEQUENCE_RED_ALERT_LEN                                                           \
    (sizeof(s_cal_chirp_sequence_red_alert) / sizeof(s_cal_chirp_sequence_red_alert[0]))

/* -- LED fading ---------------------------------------------------------- */
/* Interval the scheduler invokes led_fader_update() at. Every period and pause
 * below is quantised to this, and it bounds the fastest possible ramp. */
#define CAL_LED_TICK_MS ((uint16_t) 10)

/* Full-scale PWM count (the analogWrite() range is 0..255). */
#define CAL_LED_PWM_MAX ((uint8_t) 255)

/**
 * Fade rate falls off linearly with brightness: at full-scale PWM an LED moves
 * at its channel's maximum rate, and at 0 PWM it moves at this fraction of it.
 * The gain tracks the *absolute* PWM level, not the position between a
 * channel's own min and max, so every channel shares the same curve shape.
 *
 * 100 disables the effect (constant-rate linear fade).
 */
#define CAL_LED_MIN_RATE_GAIN_PERCENT ((uint8_t) 10)

/**
 * The LED groups. CAL_LED_CHANNEL_COUNT must stay last -- it sizes both the
 * calibration table below and LedFader's runtime state array.
 */
typedef enum
{
    CAL_LED_CHANNEL_WINGS = 0,
    CAL_LED_CHANNEL_RED_ALERT,
    CAL_LED_CHANNEL_SAUCER,
    CAL_LED_CHANNEL_BRIDGE,
    CAL_LED_CHANNEL_COUNT
} cal_led_channel_id_t;

/**
 * Per-channel breathing calibration.
 *
 * @c period_ms is the *whole* cycle -- bottom to top and back, pauses
 * included -- so each ramp takes (period_ms - pause_top_ms -
 * pause_bottom_ms) / 2. A channel whose pauses leave no room to ramp is
 * clamped to one tick per ramp.
 *
 * @c min_percent / @c max_percent are percentages of the full PWM range (not
 * of each other); min == max means "hold at that level".
 */
typedef struct
{
    uint8_t pin; /* must be a PWM-capable pin */
    uint16_t period_ms;
    uint8_t min_percent;
    uint8_t max_percent;
    uint16_t pause_bottom_ms;
    uint16_t pause_top_ms;
} cal_led_channel_t;

/* Indexed by cal_led_channel_id_t -- keep this in the enum's order. Columns are
 * hand-aligned so the table reads as a table; clang-format would collapse it.
 *
 * Red alert asks for a 1 s fade in-and-out with a 2 s pause at the bottom.
 * Since period_ms is the whole cycle with pauses included, that is a 3000 ms
 * period against a 2000 ms bottom pause, which leaves 500 ms per ramp. */
/* clang-format off */
static const cal_led_channel_t s_cal_led_channels[CAL_LED_CHANNEL_COUNT] = {
    /* pin                     period   min%  max%  pause_bot  pause_top */
    { CAL_PIN_LED_WINGS,         5000,    20,  20,         0,      2000 },
    { CAL_PIN_LED_RED_ALERT,     2000,     0,  100,      500,         0 },
    { CAL_PIN_LED_SAUCER,        1000,   100,  100,         0,         0 },
    { CAL_PIN_LED_BRIDGE,       10000,    2,  80,      2000,         0 }
};
/* clang-format on */

/* -- LED scenes ---------------------------------------------------------- */
/**
 * Which channels are lit in each scene. A channel in the scene's mask runs its
 * breathing cycle; a channel outside it ramps down to off on the same rate
 * curve and stays dark. Every channel keeps its own calibration above in every
 * scene -- a scene only decides who is lit, never how.
 */
typedef enum
{
    CAL_LED_SCENE_OFF = 0,
    CAL_LED_SCENE_NORMAL,
    CAL_LED_SCENE_RED_ALERT,
    CAL_LED_SCENE_COUNT
} cal_led_scene_id_t;

#define CAL_LED_CHANNEL_BIT(channel_id) ((uint8_t) (1U << (channel_id)))

/* Indexed by cal_led_scene_id_t -- keep this in that enum's order. */
/* clang-format off */
static const uint8_t s_cal_led_scene_masks[CAL_LED_SCENE_COUNT] = {
    /* CAL_LED_SCENE_OFF: everything ramps down. */
    0U,
    /* CAL_LED_SCENE_NORMAL: everything but the red alert strip. */
    (uint8_t) (CAL_LED_CHANNEL_BIT(CAL_LED_CHANNEL_WINGS) |
               CAL_LED_CHANNEL_BIT(CAL_LED_CHANNEL_SAUCER) |
               CAL_LED_CHANNEL_BIT(CAL_LED_CHANNEL_BRIDGE)),
    /* CAL_LED_SCENE_RED_ALERT: wings and saucer stay lit at their red alert
     * levels, the red alert strip joins in and the bridge goes dark. */
    (uint8_t) (CAL_LED_CHANNEL_BIT(CAL_LED_CHANNEL_WINGS) |
               CAL_LED_CHANNEL_BIT(CAL_LED_CHANNEL_SAUCER) |
               CAL_LED_CHANNEL_BIT(CAL_LED_CHANNEL_RED_ALERT))
};
/* clang-format on */

/* -- Red alert levels ---------------------------------------------------- */
/* These override the channel's own calibration while the red alert scene is
 * active; the normal scene keeps the values in the table above. */
/* The wings and saucer both fade up to the high level, then flicker at random
 * intervals between the high and low levels, so they read as a fault rather
 * than a steady light. */
#define CAL_RED_ALERT_FLICKER_HIGH_PERCENT ((uint8_t) 4)
#define CAL_RED_ALERT_FLICKER_LOW_PERCENT  ((uint8_t) 1)
/* Each flicker state lasts a random time in this range. */
#define CAL_RED_ALERT_SAUCER_FLICKER_MIN_MS ((uint16_t) 20)
#define CAL_RED_ALERT_SAUCER_FLICKER_MAX_MS ((uint16_t) 200)

#endif /* CALIBRATION_H */
