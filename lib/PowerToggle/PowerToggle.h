/**
 * @file PowerToggle.h
 *
 * @brief Header file for PowerToggle.cpp
 */

#ifndef POWER_TOGGLE_H
#define POWER_TOGGLE_H

/* System headers */
#include <stdbool.h>

/* Third-party header files */
/* None */

/* Project headers */
/* None */

/* Constants, Datatypes */
/**
 * What the last accepted touch did. POWER_TOGGLE_TURNED_ON covers both powering
 * up from off and dropping back out of red alert -- either way the model ends up
 * in its normal on state.
 */
typedef enum
{
    POWER_TOGGLE_NONE = 0,
    POWER_TOGGLE_TURNED_ON,
    POWER_TOGGLE_TURNED_OFF,
    POWER_TOGGLE_TURNED_RED_ALERT
} power_toggle_transition_t;

/* Function prototypes */
void power_toggle_init(void);
void power_toggle_update(void);
bool power_toggle_is_on(void);
bool power_toggle_is_red_alert(void);
power_toggle_transition_t power_toggle_consume_transition(void);

#endif /* POWER_TOGGLE_H */
