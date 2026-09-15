/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef __US_TICKER_DATA_H
#define __US_TICKER_DATA_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32g4xx.h"
#include "stm32g4xx_ll_tim.h"
#include "cmsis_nvic.h"

#if !defined(MBED_US_TIMER_TIM) || !defined(MBED_US_TIMER_USCORE_TIM) || !defined(MBED_US_TIMER_TIM_USCORE)

#undef MBED_US_TIMER_TIM
#undef MBED_US_TIMER_USCORE_TIM
#undef MBED_US_TIMER_TIM_USCORE

#define MBED_US_TIMER_TIM TIM5
#define MBED_US_TIMER_USCORE_TIM _TIM5
#define MBED_US_TIMER_TIM_USCORE TIM5_

#endif

#ifndef TIM_MST_BIT_WIDTH
#define TIM_MST_BIT_WIDTH  32 // 16 or 32
#endif

#define _MBED_TIMER_CAT(a, b) _MBED_TIMER_CAT_I(a, b)
#define _MBED_TIMER_CAT_I(a, b) a ## b

#define _MBED_TIMER_HAL_CALL(tim_uscore, suffix) _MBED_TIMER_HAL_CALL_I(tim_uscore, suffix)
#define _MBED_TIMER_HAL_CALL_I(tim_uscore, suffix) __HAL_RCC_ ## tim_uscore ## suffix

#define _MBED_TIMER_IRQn_SUFFIX(tim_uscore)  _MBED_TIMER_IRQn_SUFFIX_I(tim_uscore)
#define _MBED_TIMER_IRQn_SUFFIX_I(tim_uscore)  tim_uscore ## IRQn

#define TIM_MST      MBED_US_TIMER_TIM
#ifndef TIM_MST_IRQ
#define TIM_MST_IRQ  _MBED_TIMER_IRQn_SUFFIX(MBED_US_TIMER_TIM_USCORE)
#endif
#define TIM_MST_RCC  _MBED_TIMER_HAL_CALL(MBED_US_TIMER_TIM_USCORE, CLK_ENABLE) ()
#define TIM_MST_DBGMCU_FREEZE  _MBED_TIMER_CAT(__HAL_DBGMCU_FREEZE, MBED_US_TIMER_USCORE_TIM) ()

#define TIM_MST_RESET_ON   _MBED_TIMER_HAL_CALL(MBED_US_TIMER_TIM_USCORE, FORCE_RESET) ()
#define TIM_MST_RESET_OFF  _MBED_TIMER_HAL_CALL(MBED_US_TIMER_TIM_USCORE, RELEASE_RESET)()

#define TIM_MST_PCLK  1 // Select the peripheral clock number (1 or 2)

#ifdef __cplusplus
}
#endif

#endif // __US_TICKER_DATA_H
