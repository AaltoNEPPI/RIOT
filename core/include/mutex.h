/*
 * Copyright (C) 2015 Kaspar Schleiser <kaspar@schleiser.de>
 *               2013, 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    core_sync Synchronization
 * @brief       Mutex for thread synchronization
 * @ingroup     core
 * @{
 *
 * @file
 * @brief       RIOT synchronization API
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 */

#ifndef MUTEX_H
#define MUTEX_H

#include <stddef.h>

#include "list.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief Mutex structure. Must never be modified by the user.
 */
typedef struct {
    /**
     * @brief   The process waiting queue of the mutex. **Must never be changed
     *          by the user.**
     * @internal
     */
    list_node_t queue;
} mutex_t;

/**
 * @brief Static initializer for mutex_t.
 * @details This initializer is preferable to mutex_init().
 */
#define MUTEX_INIT { { NULL } }

/**
 * @brief Static initializer for mutex_t with a locked mutex
 */
#define MUTEX_INIT_LOCKED { { MUTEX_LOCKED } }

/**
 * @cond INTERNAL
 * @brief This is the value of the mutex when locked and no threads are waiting
 *        for it
 */
#define MUTEX_LOCKED ((list_node_t *)-1)
/**
 * @endcond
 */

/**
 * @brief Initializes a mutex object.
 * @details For initialization of variables use MUTEX_INIT instead.
 *          Only use the function call for dynamically allocated mutexes.
 * @param[out] mutex    pre-allocated mutex structure, must not be NULL.
 */
static inline void mutex_init(mutex_t *mutex)
{
    mutex->queue.next = NULL;
}

/**
 * @brief Lock a mutex, blocking or non-blocking.
 *
 * @details For commit purposes you should probably use mutex_trylock() and
 *          mutex_lock() instead.
 *
 * @param[in] mutex         Mutex object to lock. Has to be initialized first.
 *                          Must not be NULL.
 * @param[in] blocking      if true, block until mutex is available.
 *
 * @return 1 if mutex was unlocked, now it is locked.
 * @return 0 if the mutex was locked.
 */
int _mutex_lock(mutex_t *mutex, int blocking);

/**
 * @brief Tries to get a mutex, non-blocking.
 *
 * @param[in] mutex Mutex object to lock. Has to be initialized first. Must not
 *                  be NULL.
 *
 * @return 1 if mutex was unlocked, now it is locked.
 * @return 0 if the mutex was locked.
 */
static inline int mutex_trylock(mutex_t *mutex)
{
    return _mutex_lock(mutex, 0);
}

/**
 * @brief Locks a mutex, blocking.
 *
 * @param[in] mutex Mutex object to lock. Has to be initialized first. Must not be NULL.
 */
static inline void mutex_lock(mutex_t *mutex)
{
    _mutex_lock(mutex, 1);
}

/**
 * @brief Unlocks the mutex.
 *
 * @param[in] mutex Mutex object to unlock, must not be NULL.
 */
void mutex_unlock(mutex_t *mutex);

/**
 * @brief Unlocks the mutex and sends the current thread to sleep
 *
 * @param[in] mutex Mutex object to unlock, must not be NULL.
 */
void mutex_unlock_and_sleep(mutex_t *mutex);

/**
 * @brief   Puts the current thread into sleep, holding the given mutex.
 *
 * @details The thread must hold the mutex when called.
 *          The thread must be woken externally with thread_wakeup_mutex().
 *          A benefit of this is that the waker does not need to know
 *          what thread is sleeping; knowing the mutex is enough.
 *
 *          If the interrupts are disabled when called, enables the
 *          interrupts while the thread sleeps and disables them
 *          again before returning.
 *
 *          The mutex remains locked while the thread sleeps.
 */
void mutex_thread_sleep(mutex_t *m);

/**
 * @brief   Wakes up a thread sleeping while keeping a mutex.
 *
 * @param[in] mutex the mutex the sleeping thread is sleeping on.
 *
 * @details Typically called from an interrupt context.  If the
 *          interrupts where disabled when the thread called
 *          thread_sleep_keep_mutex(), makes sure that interrupts are
 *          disabled when sleeping thread is resumed.
 * 
 *          Note, however, that if there were or are higher priority
 *          threads running, those is allowed to run before the
 *          sleeping thread is resumed.  Hence, this does *not*
 *          guarantee that nothing happens between this call and
 *          resuming, unless there are no other higher priority
 *          interrupts and the sleeping thread has the highest
 *          priority.
 */
void mutex_thread_wakeup(mutex_t *mutex);

#ifdef __cplusplus
}
#endif

#endif /* MUTEX_H */
/** @} */
