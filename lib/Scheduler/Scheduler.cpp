/**
 * @file Scheduler.cpp
 *
 * @brief Minimal cooperative task scheduler.
 *
 * Tasks never block and never preempt one another -- each registered task is
 * expected to check its own state and return quickly. scheduler_run() polls
 * every registered task once per call and dispatches those whose period has
 * elapsed, so it must itself be called continuously from loop().
 */

/* File header */
#include "Scheduler.h"

/* System headers */
#include <Arduino.h>

/* Third-party header files */
/* None */

/* Project headers */
/* None */

/* Constants, macros, datatypes */
#define SCHEDULER_MAX_TASKS 8

typedef struct
{
    scheduler_task_fn_t task_fn;
    uint16_t period_ms;
    unsigned long last_run_ms;
} scheduler_task_t;

/* Static variable definitions */
static scheduler_task_t s_tasks[SCHEDULER_MAX_TASKS];
static uint8_t s_task_count = 0;

/* Static function prototypes */
/* None */

/**
 * @brief Reset the scheduler to an empty task table.
 */
void scheduler_init(void) { s_task_count = 0; }

/**
 * @brief Register a task to be polled by scheduler_run().
 *
 * @param task_fn    Function to invoke when the task's period elapses.
 * @param period_ms  Minimum interval between calls. 0 runs the task on
 *                    every scheduler_run() pass (for latency-sensitive work).
 * @return true if the task was registered, false if the task table is full.
 */
bool scheduler_add_task(scheduler_task_fn_t task_fn, uint16_t period_ms)
{
    if (s_task_count >= SCHEDULER_MAX_TASKS)
    {
        return false;
    }

    s_tasks[s_task_count].task_fn     = task_fn;
    s_tasks[s_task_count].period_ms   = period_ms;
    s_tasks[s_task_count].last_run_ms = 0;
    s_task_count++;

    return true;
}

/**
 * @brief Dispatch every registered task whose period has elapsed.
 *
 * Must be called continuously (e.g. from loop()) with nothing else in the
 * call path that blocks, or every task's timing suffers.
 */
void scheduler_run(void)
{
    unsigned long now_ms = millis();

    for (uint8_t i = 0; i < s_task_count; i++)
    {
        scheduler_task_t *task = &s_tasks[i];

        if ((task->period_ms == 0) || (now_ms - task->last_run_ms >= task->period_ms))
        {
            task->last_run_ms = now_ms;
            task->task_fn();
        }
    }
}
