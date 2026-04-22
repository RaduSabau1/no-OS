/***************************************************************************//**
 *   @file   foc_example.c
 *   @brief  Full FOC control loop example for BLDC_App.
 *
 *           This example runs the complete Field Oriented Control chain
 *           entirely on the STM32F7, without any TMC4671 involvement:
 *
 *           Position / velocity feedback:
 *             Optical encoder on TIM2 (encoder mode, x4 quadrature).
 *             TIM2->CNT read directly in the FOC tick — no interrupt overhead.
 *             Electrical angle derived from count modulo counts_per_elec_rev.
 *             Velocity derived from count delta over the speed loop period.
 *
 *           Startup — self-aligning:
 *             Encoder counter is zeroed at power-on (arbitrary position).
 *             FOC starts immediately with θ = 0 assumed. The control loop
 *             naturally attracts the rotor toward the θ = 0 equilibrium over
 *             the first few electrical cycles, performing alignment implicitly.
 *             Speed reference is ramped from 0 — not a design choice unique to
 *             this system, but a basic requirement of any rotating machine that
 *             cannot instantaneously change speed (inertia + back-EMF physics).
 *
 *           Feedback path (per current loop tick):
 *             ADC injected (Ia, Ib) → Clarke → Park → Id, Iq
 *
 *           Forward path (per current loop tick):
 *             Speed PI:  ω_ref – ω_meas   → Iq_ref
 *             Torque PI: Iq_ref – Iq       → Vq
 *             Flux PI:   0 – Id            → Vd
 *             Inverse Park: (Vd, Vq, θ)   → (Vα, Vβ)
 *             SVPWM: (Vα, Vβ)             → (ta, tb, tc)
 *             motor_pwm_set_duty(ta, tb, tc)
 *
 *           ADC injection-complete ISR drives the FOC tick (TODO).
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
#include "foc_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "motor_pwm.h"
#include "vector_transfs.h"
#include "fp_pid.h"
#include "svpwm.h"
#include "no_os_print_log.h"
#include <math.h>

/*
 * Encoder — TIM2 configured in encoder mode (TIM_ENCODERMODE_TI12, x4
 * quadrature). TIM2 on STM32F7 has a 32-bit counter so wrap-around
 * handling is needed only for very long runs.
 * TODO: configure TIM2 in the .ioc and ensure htim2 is initialised before
 *       foc_example_main() is called.
 */
extern TIM_HandleTypeDef htim2;

/* DC bus voltage [V] — measure or read from ADC in final implementation. */
#define VDC_BUS_V			24.0f

/* Target speed [rad/s electrical] */
#define REF_SPEED_RAD_S			100.0f

/*
 * Speed reference ramp rate [rad/s per speed-loop tick].
 * A ramp is mandatory for any rotating machine — instantaneous speed steps
 * are not physically achievable and would cause current saturation.
 * This is baseline practice, not a design contribution.
 * TODO: tune for the specific motor / load inertia.
 */
#define SPEED_RAMP_RATE			0.5f

/* Encoder resolution — update to match the actual encoder datasheet. */
#define ENCODER_PPR			1000	/* TODO: set to actual PPR */
#define COUNTS_PER_MECH_REV		(ENCODER_PPR * 4)   /* x4 quadrature */
#define COUNTS_PER_ELEC_REV		(COUNTS_PER_MECH_REV / MOTOR_POLE_PAIRS)

/* Current loop sample time [s] — 25 kHz switching → 40 µs */
#define DT_CURRENT_S			0.00004f

/* Speed loop runs every N current loop ticks */
#define SPEED_LOOP_DIVIDER		20
#define DT_SPEED_S			(DT_CURRENT_S * SPEED_LOOP_DIVIDER)

/**
 * @brief Full FOC control loop — all transforms on STM32F7, encoder feedback.
 * @return 0 on success, negative error code otherwise.
 */
int foc_example_main()
{
	struct tmc6100_desc  *tmc6100_desc;
	struct motor_pwm_desc *motor_pwm_desc;

	struct sFFClarke clarke;
	struct sFPark    fpark;
	struct sIPark    ipark;
	struct svpwm_out svpwm;

	float theta, omega;
	float ref_speed = 0.0f;		/* ramped up from 0 at startup */
	float ia, ib;			/* Phase currents [A] — from ADC */

	uint32_t enc_count, prev_enc_count = 0;
	int32_t  delta_counts;
	int speed_div = 0;
	int ret;

	/* Speed outer loop PI — anti-windup via output clamping */
	struct sPI speed_pi = {
		.fDtSec     = DT_SPEED_S,
		.fKp        = 1.0f,		/* TODO: tune */
		.fKi        = 0.01f,		/* TODO: tune */
		.fUpOutLim  =  20.0f,		/* Iq_ref max [A] */
		.fLowOutLim = -20.0f,
	};

	/* Torque (Iq) inner loop PI */
	struct sPI torque_pi = {
		.fDtSec     = DT_CURRENT_S,
		.fKp        = 0.5f,		/* TODO: tune */
		.fKi        = 50.0f,		/* TODO: tune */
		.fUpOutLim  =  VDC_BUS_V * 0.5f,
		.fLowOutLim = -VDC_BUS_V * 0.5f,
	};

	/* Flux (Id) inner loop PI — Id reference = 0 for surface-mounted BLDC */
	struct sPI flux_pi = {
		.fDtSec     = DT_CURRENT_S,
		.fKp        = 0.5f,		/* TODO: tune */
		.fKi        = 50.0f,		/* TODO: tune */
		.fUpOutLim  =  VDC_BUS_V * 0.5f,
		.fLowOutLim = -VDC_BUS_V * 0.5f,
	};

	/* --- Hardware init -------------------------------------------------- */

	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
	if (ret)
		goto exit;

	ret = motor_pwm_init(&motor_pwm_desc, &motor_pwm_ip);
	if (ret)
		goto remove_tmc6100;

	/*
	 * TIM2 encoder mode assumed already configured by HAL (htim2).
	 * Counter starts at 0 — arbitrary rotor position.
	 * The FOC loop self-aligns over the first electrical cycles.
	 */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	/* TODO: configure ADC injected channels triggered by TIM1 TRGO.
	 *       Replace polling loop below with ADC-ISR-driven tick. */

	ret = motor_pwm_enable(motor_pwm_desc);
	if (ret)
		goto remove_pwm;

	/* --- Control loop --------------------------------------------------- */
	while (1) {

		/* 1. Read encoder counter — electrical angle */
		enc_count = __HAL_TIM_GET_COUNTER(&htim2);
		theta = (float)(enc_count % (uint32_t)COUNTS_PER_ELEC_REV)
			* (2.0f * (float)M_PI / (float)COUNTS_PER_ELEC_REV);

		/* 2. Read phase currents (TODO: replace with ADC injected API) */
		ia = 0.0f;	/* TODO: read from ADC injected channel */
		ib = 0.0f;	/* TODO: read from ADC injected channel */

		/* 3. Clarke: (Ia, Ib) → (Iα, Iβ) */
		clarke.fA = ia;
		clarke.fB = ib;
		clarke.fC = -(ia + ib);
		tFFClarke_abc2albe(&clarke);

		/* 4. Park: (Iα, Iβ, θ) → (Id, Iq) */
		fpark.fAl    = clarke.fAl;
		fpark.fBe    = clarke.fBe;
		fpark.fCosAng = cosf(theta);
		fpark.fSinAng = sinf(theta);
		tFPark_albe2dq(&fpark);

		/* 5. Outer speed loop — runs every SPEED_LOOP_DIVIDER ticks */
		if (++speed_div >= SPEED_LOOP_DIVIDER) {
			speed_div = 0;

			/* Velocity from encoder count delta */
			delta_counts = (int32_t)(enc_count - prev_enc_count);
			prev_enc_count = enc_count;
			omega = (float)delta_counts
				* (2.0f * (float)M_PI)
				/ ((float)COUNTS_PER_ELEC_REV * DT_SPEED_S);

			/* Speed reference ramp — standard rotating-machine
			 * requirement, not a design contribution. */
			if (ref_speed < REF_SPEED_RAD_S)
				ref_speed += SPEED_RAMP_RATE;

			speed_pi.fIn = ref_speed - omega;
			tPI_calc(&speed_pi);
		}

		/* 6. Inner current loops */
		torque_pi.fIn = speed_pi.fOut - fpark.fQ;
		flux_pi.fIn   = 0.0f        - fpark.fD;
		tPI_calc(&torque_pi);
		tPI_calc(&flux_pi);

		/* 7. Inverse Park: (Vd, Vq, θ) → (Vα, Vβ) */
		ipark.fD      = flux_pi.fOut;
		ipark.fQ      = torque_pi.fOut;
		ipark.fCosAng = fpark.fCosAng;
		ipark.fSinAng = fpark.fSinAng;
		tIPark_dq2albe(&ipark);

		/* 8. SVPWM: (Vα, Vβ) → CCR ticks */
		svpwm_compute(ipark.fAl, ipark.fBe, VDC_BUS_V,
			      MOTOR_PWM_PERIOD_TICKS, &svpwm);

		/* 9. Apply duty cycles */
		ret = motor_pwm_set_duty(motor_pwm_desc,
					 svpwm.ta, svpwm.tb, svpwm.tc);
		if (ret)
			goto stop_motor;
	}

stop_motor:
	motor_pwm_disable(motor_pwm_desc);
remove_pwm:
	motor_pwm_remove(motor_pwm_desc);
remove_tmc6100:
	tmc6100_remove(tmc6100_desc);
exit:
	if (ret)
		pr_info("Error: %d\n", ret);
	return ret;
}
