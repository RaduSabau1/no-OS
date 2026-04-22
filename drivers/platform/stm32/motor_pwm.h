/***************************************************************************//**
 *   @file   stm32/motor_pwm.h
 *   @brief  Three-phase complementary PWM driver for STM32 motor control.
 *
 *           Inspired by stm32_pwm.h but purpose-built for FOC: centre-aligned
 *           counting, three complementary pairs (TIMx_CHx / TIMx_CHxN),
 *           hardware dead-time insertion (BDTR), and TRGO output for
 *           synchronised ADC injected-channel triggering.
 *
 *           Only TIM1 and TIM8 (advanced-control timers) support the full
 *           feature set required here on STM32F7.
 *
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef MOTOR_PWM_H_
#define MOTOR_PWM_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32_hal.h"

/**
 * @struct motor_pwm_init_param
 * @brief  Initialisation parameters for the three-phase motor PWM driver.
 */
struct motor_pwm_init_param {
	/** Pointer to an advanced-control timer handle (TIM1 or TIM8). */
	void *htimer;
	/** Switching period [ns]. Determines carrier frequency (e.g. 50000 → 20 kHz). */
	uint32_t period_ns;
	/** Dead-time inserted between high-side turn-off and low-side turn-on [ns]. */
	uint16_t deadtime_ns;
	/** When true, configure TRGO = TIM_TRGO_UPDATE to trigger ADC at period centre. */
	bool trgo_enable;
	/** Pointer to a function returning the timer input clock frequency [Hz]. */
	uint32_t (*get_timer_clock)(void);
};

/**
 * @struct motor_pwm_desc
 * @brief  Runtime descriptor for the three-phase motor PWM driver.
 */
struct motor_pwm_desc {
	/** Pointer to the advanced-control timer handle in use. */
	void *htimer;
	/** Auto-reload register value (period in timer ticks). */
	uint32_t period_ticks;
	/** Dead-time in timer ticks (written to BDTR.DTG). */
	uint16_t deadtime_ticks;
};

/**
 * @brief Initialise the three-phase motor PWM.
 *
 * Configures the advanced-control timer in centre-aligned mode with
 * three complementary output pairs and hardware dead-time insertion.
 * If trgo_enable is set, TRGO is configured to fire at the PWM period
 * update event (centre of carrier) for synchronised ADC sampling.
 *
 * @param desc   Output descriptor pointer.
 * @param param  Initialisation parameters.
 * @return 0 on success, negative error code otherwise.
 */
int motor_pwm_init(struct motor_pwm_desc **desc,
		   struct motor_pwm_init_param *param);

/**
 * @brief Release resources and disable PWM outputs.
 * @param desc  Descriptor to free.
 * @return 0 on success, negative error code otherwise.
 */
int motor_pwm_remove(struct motor_pwm_desc *desc);

/**
 * @brief Atomically update the compare registers for all three phases.
 *
 * Values are written directly to TIMx CCR1/CCR2/CCR3 and take effect at the
 * next timer update event (centre of next PWM period). The caller is
 * responsible for ensuring values are in [0, ARR]; svpwm_compute() guarantees
 * this when called with the same period_ticks as the timer ARR.
 *
 * @param desc  Driver descriptor.
 * @param ta    Phase A CCR value [ticks].
 * @param tb    Phase B CCR value [ticks].
 * @param tc    Phase C CCR value [ticks].
 * @return 0 on success, negative error code otherwise.
 */
int motor_pwm_set_duty(struct motor_pwm_desc *desc,
		       uint16_t ta, uint16_t tb, uint16_t tc);

/**
 * @brief Enable PWM outputs (sets MOE bit in BDTR, starts all channels).
 * @param desc  Driver descriptor.
 * @return 0 on success, negative error code otherwise.
 */
int motor_pwm_enable(struct motor_pwm_desc *desc);

/**
 * @brief Disable PWM outputs (clears MOE bit, all outputs go to idle state).
 * @param desc  Driver descriptor.
 * @return 0 on success, negative error code otherwise.
 */
int motor_pwm_disable(struct motor_pwm_desc *desc);

#endif /* MOTOR_PWM_H_ */
