/***************************************************************************//**
 *   @file   stm32/motor_pwm.c
 *   @brief  Three-phase complementary PWM driver for STM32 motor control.
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
#include "motor_pwm.h"
#include "no_os_alloc.h"
#include "no_os_error.h"

/* TODO: implement using STM32 HAL TIM advanced-control API.
 *
 * Key configuration steps:
 *   1. Set TIM_CR1.CMS = 10b (centre-aligned mode 2, count up+down).
 *   2. Set ARR = period_ticks = (timer_clock / (2 * f_sw)) - 1.
 *   3. Configure OC mode PWM1 on CH1, CH2, CH3 with complementary outputs.
 *   4. Write BDTR.DTG with the dead-time value (deadtime_ticks).
 *   5. If trgo_enable: set CR2.MMS = 010b (TIM_TRGO_UPDATE).
 *   6. Call HAL_TIMEx_PWMN_Start / HAL_TIM_PWM_Start for all channels.
 *   7. Set MOE (BDTR bit 15) to enable main output.
 */

int motor_pwm_init(struct motor_pwm_desc **desc,
		   struct motor_pwm_init_param *param)
{
	struct motor_pwm_desc *d;
	uint32_t timer_clk;

	if (!desc || !param || !param->htimer || !param->get_timer_clock)
		return -EINVAL;

	d = no_os_calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;

	timer_clk = param->get_timer_clock();

	/*
	 * Centre-aligned: ARR = timer_clk / (2 * f_sw) - 1
	 * period_ns → f_sw = 1e9 / period_ns
	 * period_ticks = timer_clk * period_ns / (2 * 1e9)
	 */
	d->period_ticks = (uint32_t)((uint64_t)timer_clk *
				     param->period_ns / 2000000000ULL);
	d->deadtime_ticks = (uint16_t)((uint64_t)timer_clk *
				       param->deadtime_ns / 1000000000ULL);
	d->htimer = param->htimer;

	/* TODO: apply configuration to the HAL timer handle. */

	*desc = d;
	return 0;
}

int motor_pwm_remove(struct motor_pwm_desc *desc)
{
	if (!desc)
		return -EINVAL;

	/* TODO: stop timer, disable outputs. */

	no_os_free(desc);
	return 0;
}

int motor_pwm_set_duty(struct motor_pwm_desc *desc,
		       uint16_t ta, uint16_t tb, uint16_t tc)
{
	TIM_TypeDef *tim;

	if (!desc || !desc->htimer)
		return -EINVAL;

	tim = ((TIM_HandleTypeDef *)desc->htimer)->Instance;

	/* Direct CCR writes — effective at next timer update event. */
	tim->CCR1 = ta;
	tim->CCR2 = tb;
	tim->CCR3 = tc;

	return 0;
}

int motor_pwm_enable(struct motor_pwm_desc *desc)
{
	TIM_TypeDef *tim;

	if (!desc || !desc->htimer)
		return -EINVAL;

	tim = ((TIM_HandleTypeDef *)desc->htimer)->Instance;

	/* TODO: start complementary channels via HAL_TIMEx_PWMN_Start.
	 * Set MOE (main output enable). */
	tim->BDTR |= TIM_BDTR_MOE;

	return 0;
}

int motor_pwm_disable(struct motor_pwm_desc *desc)
{
	TIM_TypeDef *tim;

	if (!desc || !desc->htimer)
		return -EINVAL;

	tim = ((TIM_HandleTypeDef *)desc->htimer)->Instance;

	tim->BDTR &= ~TIM_BDTR_MOE;

	return 0;
}
