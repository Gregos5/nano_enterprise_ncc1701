/**
 * @file ChirpNotifier.h
 *
 * @brief Header file for ChirpNotifier.cpp
 */

#ifndef CHIRP_NOTIFIER_H
#define CHIRP_NOTIFIER_H

/* System headers */
/* None */

/* Third-party header files */
/* None */

/* Project headers */
/* None */

/* Constants, Datatypes */
/* The note tables these two names select between live in Calibration.h. */
typedef enum
{
    CHIRP_SEQUENCE_ON = 0,
    CHIRP_SEQUENCE_OFF,
    CHIRP_SEQUENCE_RED_ALERT
} chirp_sequence_t;

/* Function prototypes */
void chirp_notifier_init(void);
void chirp_notifier_update(void);
void chirp_notifier_play(chirp_sequence_t sequence);

#endif /* CHIRP_NOTIFIER_H */
