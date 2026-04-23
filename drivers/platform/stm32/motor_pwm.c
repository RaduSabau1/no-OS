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

int motor_pwm_init(struct motor_pwm_desc **desc,
		   struct motor_pwm_init_param *param)
{
	TIM_OC_InitTypeDef oc_config = {0};
	TIM_BreakDeadTimeConfigTypeDef bdtr_config = {0};
	TIM_MasterConfigTypeDef master_config = {0};
	TIM_HandleTypeDef *htimer;
	struct motor_pwm_desc *d;
	uint32_t timer_clk;
	int ret;

	if (!desc || !param || !param->htimer || !param->get_timer_clock)
		return -EINVAL;

	d = no_os_calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;

	timer_clk = param->get_timer_clock();
	htimer = (TIM_HandleTypeDef *)param->htimer;

	/*
	 * Centre-aligned ARR: f_sw = f_clk / (2 * ARR)
	 *   → period_ticks = f_clk * period_ns / (2 * 1e9)
	 * Dead-time: ticks = f_clk * deadtime_ns / 1e9
	 * DTG field is 8-bit; values < 128 map linearly (1 tick resolution).
	 * At 216 MHz and 200 ns: deadtime_ticks = 43, well within range.
	 */
	d->period_ticks = (uint32_t)((uint64_t)timer_clk *
				     param->period_ns / 2000000000ULL);
	d->deadtime_ticks = (uint16_t)((uint64_t)timer_clk *
				       param->deadtime_ns / 1000000000ULL);
	d->htimer = param->htimer;

	/*
	 * 1. Timer base — centre-aligned mode 2.
	 *    RepetitionCounter = 1: centre-aligned generates two overflow events
	 *    per switching period (at ARR and at 0). RCR = 1 halves the update
	 *    rate, so the update event (and therefore TRGO) fires once per full
	 *    switching period at the counter underflow (CNT = 0), which is the
	 *    ideal sampling point for FOC current measurement.
	 */
	htimer->Init.Prescaler         = 0;
	htimer->Init.CounterMode       = TIM_COUNTERMODE_CENTERALIGNED2;
	htimer->Init.Period            = d->period_ticks;
	htimer->Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htimer->Init.RepetitionCounter = 1;
	htimer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(htimer) != HAL_OK) {
		ret = -EIO;
		goto free_desc;
	}

	/*
	 * 2. Output compare — PWM mode 1 on CH1, CH2, CH3.
	 *    All three phases share the same OC config; only CCR differs at
	 *    runtime (written directly by motor_pwm_set_duty).
	 *    Initial pulse = 0 (outputs low until first motor_pwm_set_duty call).
	 */
	oc_config.OCMode       = TIM_OCMODE_PWM1;
	oc_config.Pulse        = 0;
	oc_config.OCPolarity   = TIM_OCPOLARITY_HIGH;
	oc_config.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	oc_config.OCFastMode   = TIM_OCFAST_DISABLE;
	oc_config.OCIdleState  = TIM_OCIDLESTATE_RESET;
	oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(htimer, &oc_config, TIM_CHANNEL_1) != HAL_OK ||
	    HAL_TIM_PWM_ConfigChannel(htimer, &oc_config, TIM_CHANNEL_2) != HAL_OK ||
	    HAL_TIM_PWM_ConfigChannel(htimer, &oc_config, TIM_CHANNEL_3) != HAL_OK) {
		ret = -EIO;
		goto deinit_timer;
	}

	/*
	 * 3. Break and dead-time register (BDTR).
	 *    Dead-time value written directly as DTG byte (linear mapping for
	 *    deadtime_ticks < 128). OSSR/OSSI ensure outputs go to a safe idle
	 *    state when MOE is cleared. No hardware break input used.
	 */
	bdtr_config.OSSRState       = TIM_OSSR_ENABLE;
	bdtr_config.OSSIState       = TIM_OSSI_ENABLE;
	bdtr_config.LockLevel       = TIM_LOCKLEVEL_OFF;
	bdtr_config.DeadTime        = (uint8_t)d->deadtime_ticks;
	bdtr_config.BreakState      = TIM_BREAK_DISABLE;
	bdtr_config.BreakPolarity   = TIM_BREAKPOLARITY_HIGH;
	bdtr_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(htimer, &bdtr_config) != HAL_OK) {
		ret = -EIO;
		goto deinit_timer;
	}

	/*
	 * 4. TRGO = UPDATE → ADC injected trigger.
	 *    With RCR = 1, the update event fires at CNT = 0 (carrier valley),
	 *    triggering the ADC to sample phase currents at the switching noise
	 *    minimum.
	 */
	if (param->trgo_enable) {
		master_config.MasterOutputTrigger = TIM_TRGO_UPDATE;
		master_config.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(htimer,
							  &master_config) != HAL_OK) {
			ret = -EIO;
			goto deinit_timer;
		}
	}

	*desc = d;
	return 0;

deinit_timer:
	HAL_TIM_PWM_DeInit(htimer);
free_desc:
	no_os_free(d);
	return ret;
}

int motor_pwm_remove(struct motor_pwm_desc *desc)
{
	TIM_HandleTypeDef *htimer;

	if (!desc)
		return -EINVAL;

	htimer = (TIM_HandleTypeDef *)desc->htimer;

	/* Stop all six switch outputs then deinitialise the timer. */
	HAL_TIMEx_PWMN_Stop(htimer, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(htimer, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(htimer, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(htimer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(htimer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(htimer, TIM_CHANNEL_3);
	HAL_TIM_PWM_DeInit(htimer);

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
	TIM_HandleTypeDef *htimer;

	if (!desc || !desc->htimer)
		return -EINVAL;

	htimer = (TIM_HandleTypeDef *)desc->htimer;

	/*
	 * Start main (CHx) and complementary (CHxN) outputs for all three
	 * phases. HAL_TIMEx_PWMN_Start sets MOE internally on the last call,
	 * enabling all six gate signals simultaneously.
	 */
	if (HAL_TIM_PWM_Start(htimer, TIM_CHANNEL_1) != HAL_OK ||
	    HAL_TIM_PWM_Start(htimer, TIM_CHANNEL_2) != HAL_OK ||
	    HAL_TIM_PWM_Start(htimer, TIM_CHANNEL_3) != HAL_OK ||
	    HAL_TIMEx_PWMN_Start(htimer, TIM_CHANNEL_1) != HAL_OK ||
	    HAL_TIMEx_PWMN_Start(htimer, TIM_CHANNEL_2) != HAL_OK ||
	    HAL_TIMEx_PWMN_Start(htimer, TIM_CHANNEL_3) != HAL_OK)
		return -EIO;

	return 0;
}

int motor_pwm_disable(struct motor_pwm_desc *desc)
{
	TIM_TypeDef *tim;

	if (!desc || !desc->htimer)
		return -EINVAL;

	tim = ((TIM_HandleTypeDef *)desc->htimer)->Instance;

	/*
	 * Single register write clears MOE, atomically forcing all six outputs
	 * to their OSSI idle state in one instruction. Faster than stopping
	 * channels individually via HAL — intentional for fault response.
	 */
	tim->BDTR &= ~TIM_BDTR_MOE;

	return 0;
}
