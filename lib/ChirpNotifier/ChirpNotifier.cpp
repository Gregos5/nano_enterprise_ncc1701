/**
 * @file ChirpNotifier.cpp
 *
 * @brief Starfleet combadge chirp.
 *
 * chirp_notifier_play() requests one of two note sequences (on/off); playback
 * itself is a non-blocking state machine advanced by chirp_notifier_update()
 * -- no delay() calls -- so it cooperates cleanly with other scheduler tasks
 * instead of stalling them. A new play request cuts off whatever is
 * currently sounding, since it signals a mode change and the latest mode
 * should always win.
 *
 * The note tables themselves live in Calibration.h. CHIRP_SEQUENCE_ON's
 * frequencies/durations were extracted from tng_chirp3_clean.mp3 via STFT
 * peak-frequency analysis. This is a frequency sequence for tone(), not a raw
 * PCM sample -- a passive piezo can only reproduce single tones at a time
 * anyway, so this is the right representation, not a simplification of a
 * "real" 8-bit array.
 *
 * CHIRP_SEQUENCE_OFF is a placeholder (two descending notes) until a real
 * "power down" chirp is picked.
 */

/* File header */
#include "ChirpNotifier.h"

/* System headers */
#include <Arduino.h>

/* Third-party header files */
/* None */

/* Project headers */
#include "Calibration.h"

/* Constants, macros, datatypes */
typedef enum
{
    CHIRP_STATE_IDLE = 0,
    CHIRP_STATE_NOTE_ON,
    CHIRP_STATE_NOTE_GAP
} chirp_state_t;

/* Static variable definitions */
static bool s_play_requested                        = false;
static const cal_chirp_note_t *s_requested_sequence = s_cal_chirp_sequence_on;
static uint8_t s_requested_sequence_len             = CAL_CHIRP_SEQUENCE_ON_LEN;

static chirp_state_t s_state                     = CHIRP_STATE_IDLE;
static const cal_chirp_note_t *s_active_sequence = NULL;
static uint8_t s_active_sequence_len             = 0;
static uint8_t s_note_index                      = 0;
static unsigned long s_note_phase_start_ms       = 0;

/* Static function prototypes */
static void chirp_start_active_sequence(unsigned long now_ms);
static void chirp_advance_to_next_note(void);

/**
 * @brief Configure the buzzer pin.
 */
void chirp_notifier_init(void) { pinMode(CAL_PIN_BUZZER, OUTPUT); }

/**
 * @brief Advance the chirp playback state machine. Must be polled
 *        frequently (every scheduler pass) to keep note timing accurate.
 */
void chirp_notifier_update(void)
{
    unsigned long now_ms = millis();

    if (s_play_requested)
    {
        s_play_requested = false;
        noTone(CAL_PIN_BUZZER);
        chirp_start_active_sequence(now_ms);
        return;
    }

    switch (s_state)
    {
        case CHIRP_STATE_IDLE:
            /* Nothing to do; waiting for a play request. */
            break;

        case CHIRP_STATE_NOTE_ON:
            if (now_ms - s_note_phase_start_ms >= s_active_sequence[s_note_index].duration_ms)
            {
                noTone(CAL_PIN_BUZZER);
                s_note_phase_start_ms = now_ms;
                if (s_active_sequence[s_note_index].gap_ms > 0)
                {
                    s_state = CHIRP_STATE_NOTE_GAP;
                }
                else
                {
                    chirp_advance_to_next_note();
                }
            }
            break;

        case CHIRP_STATE_NOTE_GAP:
            if (now_ms - s_note_phase_start_ms >= s_active_sequence[s_note_index].gap_ms)
            {
                chirp_advance_to_next_note();
            }
            break;
    }
}

/**
 * @brief Request that a note sequence be (re)started. Overrides whatever is
 *        currently playing -- the latest requested sequence always wins.
 */
void chirp_notifier_play(chirp_sequence_t sequence)
{
    switch (sequence)
    {
        case CHIRP_SEQUENCE_OFF:
            s_requested_sequence     = s_cal_chirp_sequence_off;
            s_requested_sequence_len = CAL_CHIRP_SEQUENCE_OFF_LEN;
            break;

        case CHIRP_SEQUENCE_RED_ALERT:
            s_requested_sequence     = s_cal_chirp_sequence_red_alert;
            s_requested_sequence_len = CAL_CHIRP_SEQUENCE_RED_ALERT_LEN;
            break;

        case CHIRP_SEQUENCE_ON:
        default:
            s_requested_sequence     = s_cal_chirp_sequence_on;
            s_requested_sequence_len = CAL_CHIRP_SEQUENCE_ON_LEN;
            break;
    }

    s_play_requested = true;
}

/**
 * @brief Begin playback of the most recently requested sequence from its
 *        first note.
 */
static void chirp_start_active_sequence(unsigned long now_ms)
{
    s_active_sequence     = s_requested_sequence;
    s_active_sequence_len = s_requested_sequence_len;
    s_note_index          = 0;

    if (s_active_sequence_len == 0)
    {
        s_state = CHIRP_STATE_IDLE;
        return;
    }

    s_note_phase_start_ms = now_ms;
    tone(CAL_PIN_BUZZER, s_active_sequence[s_note_index].freq_hz,
         s_active_sequence[s_note_index].duration_ms);
    s_state = CHIRP_STATE_NOTE_ON;
}

/**
 * @brief Move to the next note in the active sequence, or back to idle once
 *        the sequence is complete.
 */
static void chirp_advance_to_next_note(void)
{
    s_note_index++;
    if (s_note_index >= s_active_sequence_len)
    {
        s_state = CHIRP_STATE_IDLE;
        return;
    }

    s_note_phase_start_ms = millis();
    tone(CAL_PIN_BUZZER, s_active_sequence[s_note_index].freq_hz,
         s_active_sequence[s_note_index].duration_ms);
    s_state = CHIRP_STATE_NOTE_ON;
}
