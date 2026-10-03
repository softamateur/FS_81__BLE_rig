/**
  ******************************************************************************
  * @file      stm32_lock_user.h
  * @author    STMicroelectronics
  * @brief     User defined lock mechanisms
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __STM32_LOCK_USER_H__
#define __STM32_LOCK_USER_H__

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#if STM32_LOCK_API != 1 /* Version of the implemented API */
#error stm32_lock_user.h not compatible with current version of stm32_lock.h
#endif

/* Remove the following line when you have implemented your own thread-safe
 * solution. */
// #error Please implement your own thread-safe solution

// Custom solution by Lundinova. Adapated from STM32_THREAD_SAFE_STRATEGY == 5.
// Cannot lock from interrupts (which is a normal implementation as they would block),
// however unlike strategy #5 this uses a recursive mutex instead of suspending all
// tasks. This allows higher priority tasks to run _during_ the time the lock might
// be acquired by a lower priority task.
//
// Recursive mutex is needed due to repeated lock acquisition (and release). Using 
// FreeRTOS calls directly rather than CMSIS_V2 (as ST is) and creating a static
// mutex to avoid any initialization issues (that a locking newlib functional is
// called before the mutex is initialized).
  
/*
 * Deny lock usage from interrupts. Implemented using FreeRTOS locks.
 */

/* Includes ----------------------------------------------------------------*/
#include <FreeRTOS.h>
#include <task.h>
#include "semphr.h"
#include "cmsis_os2.h"
// extern osMutexId_t C_LibMutexHandle;    // CMSIS_V2
extern SemaphoreHandle_t newlib_mutex;  // FreeRTOS

#if defined (__GNUC__) && !defined (__CC_ARM) && configUSE_NEWLIB_REENTRANT == 0
#warning Please set configUSE_NEWLIB_REENTRANT to 1 in FreeRTOSConfig.h, otherwise newlib will not be thread-safe
#endif /* defined (__GNUC__) && !defined (__CC_ARM) && configUSE_NEWLIB_REENTRANT == 0 */

/* Private defines -----------------------------------------------------------*/
/** Initialize members in instance of <code>LockingData_t</code> structure */
#define LOCKING_DATA_INIT { /* Add fields initialization here */ }

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* Add fields here */
} LockingData_t;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief Initialize STM32 lock
  * @param lock The lock to init
  */
static inline void stm32_lock_init(LockingData_t *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);
}

/**
  * @brief Acquire STM32 lock
  * @param lock The lock to acquire
  */
static inline void stm32_lock_acquire(LockingData_t *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);
  STM32_LOCK_BLOCK_IF_INTERRUPT_CONTEXT();
  // vTaskSuspendAll();
  // osMutexAcquire(C_LibMutexHandle, osWaitForever);
  xSemaphoreTakeRecursive(newlib_mutex, portMAX_DELAY);
}

/**
  * @brief Release STM32 lock
  * @param lock The lock to release
  */
static inline void stm32_lock_release(LockingData_t *lock)
{
  STM32_LOCK_BLOCK_IF_NULL_ARGUMENT(lock);
  STM32_LOCK_BLOCK_IF_INTERRUPT_CONTEXT();
  // xTaskResumeAll();
  // osMutexRelease(C_LibMutexHandle);
  xSemaphoreGiveRecursive(newlib_mutex);
}

#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */

#endif /* __STM32_LOCK_USER_H__ */
