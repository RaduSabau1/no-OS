/***************************************************************************//**
 *   @file   svpwm.h
 *   @brief  Space Vector PWM (SVPWM) computation module.
 *
 *           Takes the stationary-frame voltage references (Vα, Vβ) produced
 *           by the Inverse Park transform, determines the active sector, and
 *           computes the three-phase duty cycles (tu, tv, tw) to be written
 *           to the motor_pwm driver.
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
#ifndef __SVPWM_H__
#define __SVPWM_H__

#include <stdint.h>

/**
 * @brief Output CCR tick values for all three motor phases.
 * Values are in timer ticks [0, period_ticks], ready for direct CCR write.
 */
struct svpwm_out {
	uint16_t tu;	/* Phase U CCR ticks */
	uint16_t tv;	/* Phase V CCR ticks */
	uint16_t tw;	/* Phase W CCR ticks */
};

/**
 * @brief Compute SVPWM CCR values from stationary-frame voltage references.
 *
 * Outputs are in timer ticks [0, period_ticks] and can be written directly
 * to TIMx->CCR1/CCR2/CCR3 without further conversion.
 *
 * @param v_alpha      Alpha voltage reference [V] (from Inverse Park).
 * @param v_beta       Beta voltage reference  [V] (from Inverse Park).
 * @param v_dc         DC bus voltage [V].
 * @param period_ticks Timer ARR value (e.g. MOTOR_PWM_PERIOD_TICKS).
 * @param out          Output CCR values, written by this function.
 */
void svpwm_compute(float v_alpha, float v_beta, float v_dc,
		   uint16_t period_ticks, struct svpwm_out *out);

#endif /* __SVPWM_H__ */
