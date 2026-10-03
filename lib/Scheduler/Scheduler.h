/**
 * @file Scheduler.h
 *
 * @brief Header file for Scheduler.cpp
 */

#ifndef SCHEDULER_H
#define SCHEDULER_H

/* System headers */
#include <stdbool.h>
#include <stdint.h>

/* Third-party header files */
/* None */

/* Project headers */
/* None */

/* Constants, Datatypes */
typedef void (*scheduler_task_fn_t)(void);

/* Function prototypes */
void scheduler_init(void);
bool scheduler_add_task(scheduler_task_fn_t task_fn, uint16_t period_ms);
void scheduler_run(void);

#endif /* SCHEDULER_H */
