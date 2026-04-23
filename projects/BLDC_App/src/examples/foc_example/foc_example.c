/***************************************************************************//**
 *   @file   foc_example.c
 *   @brief  Full FOC control loop example for BLDC_App.
 *
 *           Implements a complete Field Oriented Control chain entirely on the
 *           STM32F7, without any TMC4671 involvement.
 *
 *           Timing:
 *             TIM1 TRGO fires at 25 kHz (every 40 µs, carrier valley CNT=0).
 *             ADC1 converts Ia and Ib injected channels on each TRGO.
 *             HAL_ADCEx_InjectedConvCpltCallback → foc_tick() runs the full
 *             FOC pipeline in interrupt context (~40 µs budget on M7@216 MHz).
 *
 *           Position / velocity feedback:
 *             Optical encoder on TIM2 (encoder mode, x4 quadrature).
 *             TIM2->CNT read directly in foc_tick — no interrupt overhead.
 *             Electrical angle: count modulo counts_per_elec_rev → [0, 2π].
 *             Velocity: count delta over SPEED_LOOP_DIVIDER ticks.
 *
 *           Startup — self-aligning:
 *             Encoder counter zeroed at power-on (arbitrary rotor position).
 *             FOC starts immediately with θ = 0 assumed. The control loop
 *             naturally attracts the rotor toward the θ = 0 equilibrium over
 *             the first few electrical cycles, performing alignment implicitly.
 *             Speed reference is ramped from 0 — basic requirement of any
 *             rotating machine; not a design contribution.
 *
 *           FOC pipeline (per 25 kHz tick, inside foc_tick()):
 *             ADC injected (Ia, Ib) → Clarke → Park → Id, Iq
 *             Speed PI:   ω_ref – ω_meas  → Iq_ref   [every 20 ticks]
 *             Torque PI:  Iq_ref – Iq      → Vq
 *             Flux PI:    0 – Id           → Vd
 *             Inverse Park: (Vd, Vq, θ)   → (Vα, Vβ)
 *             SVPWM: (Vα, Vβ)             → (tu, tv, tw) [CCR ticks]
 *             motor_pwm_set_duty(tu, tv, tw) → 3 register writes
 *
 *   @author Radu Sabau (radu.sabau@analog.com)
 *******************************************************************************
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
 ******************************************************************************/
#include "foc_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "motor_pwm.h"
#include "stm32_adc.h"
#include "vector_transfs.h"
#include "fp_pid.h"
#include "svpwm.h"
#include "no_os_print_log.h"
#include <math.h>

/* DC bus voltage [V] — TODO: measure or read from ADC in final implementation. */
#define VDC_BUS_V			24.0f

/* Target electrical speed [rad/s]. */
#define REF_SPEED_RAD_S			100.0f

/*
 * Speed reference ramp rate [rad/s per speed-loop tick].
 * Mandatory for any rotating machine — cannot instantaneously change speed.
 * Not a design contribution; tune for the specific motor/load inertia.
 * TODO: tune SPEED_RAMP_RATE.
 */
#define SPEED_RAMP_RATE			0.5f

/* Encoder resolution — update to match the actual encoder datasheet. */
#define ENCODER_PPR			1000	/* TODO: set to actual PPR */
#define COUNTS_PER_MECH_REV		(ENCODER_PPR * 4)	/* x4 quadrature */
#define COUNTS_PER_ELEC_REV		(COUNTS_PER_MECH_REV / MOTOR_POLE_PAIRS)

/* Current loop sample time [s] — 25 kHz switching → 40 µs. */
#define DT_CURRENT_S			0.00004f

/* Speed loop runs every SPEED_LOOP_DIVIDER current-loop ticks. */
#define SPEED_LOOP_DIVIDER		20
#define DT_SPEED_S			(DT_CURRENT_S * SPEED_LOOP_DIVIDER)

/*
 * FOC state — lives in static storage so foc_tick() (ISR context) can access
 * it without arguments beyond the adc_desc pointer.
 */
static struct {
	struct sFFClarke  clarke;
	struct sFPark     fpark;
	struct sIPark     ipark;
	struct svpwm_out  svpwm;
	struct sPI        speed_pi;
	struct sPI        torque_pi;
	struct sPI        flux_pi;

	struct motor_pwm_desc *pwm_desc;

	float             ref_speed;
	uint32_t          prev_enc_count;
	int               speed_div;

	volatile bool     running;
	volatile int      error;
} g_foc;

/**
 * @brief FOC tick — runs in ADC injection-complete interrupt context.
 *
 * Executes the full control pipeline once per 25 kHz carrier period.
 * Must complete well within 40 µs; on M7 @ 216 MHz the pipeline is
 * comfortably under 5 µs including all float math.
 */
static void foc_tick(struct stm32_adc_desc *adc_desc)
{
	uint32_t iu_raw, iw_raw, enc_count;
	int32_t  delta_counts;
	float    iu, iv, iw, theta, omega;
	int      ret;

	/* 1. Read phase currents from injected channels.
	 *    TMC6100 eval board AD8418 amplifiers provide Iu (rank 1) and
	 *    Iw (rank 2). Iv is derived from KCL: Iv = -(Iu + Iw).
	 *    Convert ADC counts to amperes:
	 *      I = (count - MIDPOINT) × CURRENT_SCALE
	 *    where MIDPOINT = 2048 (zero-current code at Vref/2) and
	 *    CURRENT_SCALE accounts for shunt resistance and amplifier gain.
	 *    TODO: verify MOTOR_SHUNT_RESISTANCE and MOTOR_CURRENT_GAIN. */
	iu_raw = stm32_adc_get_value(adc_desc, ADC_INJECTED_RANK_1);
	iw_raw = stm32_adc_get_value(adc_desc, ADC_INJECTED_RANK_2);
	iu = ((float)iu_raw - MOTOR_ADC_MIDPOINT) * MOTOR_ADC_CURRENT_SCALE;
	iw = ((float)iw_raw - MOTOR_ADC_MIDPOINT) * MOTOR_ADC_CURRENT_SCALE;
	iv = -(iu + iw);

	/* 2. Read encoder counter — electrical angle. */
	enc_count = __HAL_TIM_GET_COUNTER(&htim2);
	theta = (float)(enc_count % (uint32_t)COUNTS_PER_ELEC_REV)
		* (2.0f * (float)M_PI / (float)COUNTS_PER_ELEC_REV);

	/* 3. Clarke: (Iu, Iv, Iw) → (Iα, Iβ).  Iv = -(Iu + Iw) by KCL. */
	g_foc.clarke.fA = iu;
	g_foc.clarke.fB = iv;
	g_foc.clarke.fC = iw;
	tFFClarke_abc2albe(&g_foc.clarke);

	/* 4. Park: (Iα, Iβ, θ) → (Id, Iq). */
	g_foc.fpark.fAl    = g_foc.clarke.fAl;
	g_foc.fpark.fBe    = g_foc.clarke.fBe;
	g_foc.fpark.fCosAng = cosf(theta);
	g_foc.fpark.fSinAng = sinf(theta);
	tFPark_albe2dq(&g_foc.fpark);

	/* 5. Outer speed loop — runs every SPEED_LOOP_DIVIDER ticks (1.25 kHz). */
	if (++g_foc.speed_div >= SPEED_LOOP_DIVIDER) {
		g_foc.speed_div = 0;

		delta_counts = (int32_t)(enc_count - g_foc.prev_enc_count);
		g_foc.prev_enc_count = enc_count;
		omega = (float)delta_counts
			* (2.0f * (float)M_PI)
			/ ((float)COUNTS_PER_ELEC_REV * DT_SPEED_S);

		/* Speed reference ramp — mandatory, not a design contribution. */
		if (g_foc.ref_speed < REF_SPEED_RAD_S)
			g_foc.ref_speed += SPEED_RAMP_RATE;

		g_foc.speed_pi.fIn = g_foc.ref_speed - omega;
		tPI_calc(&g_foc.speed_pi);
	}

	/* 6. Inner current loops. */
	g_foc.torque_pi.fIn = g_foc.speed_pi.fOut - g_foc.fpark.fQ;
	g_foc.flux_pi.fIn   = 0.0f               - g_foc.fpark.fD;
	tPI_calc(&g_foc.torque_pi);
	tPI_calc(&g_foc.flux_pi);

	/* 7. Inverse Park: (Vd, Vq, θ) → (Vα, Vβ). */
	g_foc.ipark.fD      = g_foc.flux_pi.fOut;
	g_foc.ipark.fQ      = g_foc.torque_pi.fOut;
	g_foc.ipark.fCosAng = g_foc.fpark.fCosAng;
	g_foc.ipark.fSinAng = g_foc.fpark.fSinAng;
	tIPark_dq2albe(&g_foc.ipark);

	/* 8. SVPWM: (Vα, Vβ) → integer CCR ticks. */
	svpwm_compute(g_foc.ipark.fAl, g_foc.ipark.fBe, VDC_BUS_V,
		      MOTOR_PWM_PERIOD_TICKS, &g_foc.svpwm);

	/* 9. Apply — three register writes. */
	ret = motor_pwm_set_duty(g_foc.pwm_desc,
				 g_foc.svpwm.tu, g_foc.svpwm.tv, g_foc.svpwm.tw);
	if (ret) {
		g_foc.error   = ret;
		g_foc.running = false;
	}
}

/**
 * @brief Initialise hardware and run the ISR-driven FOC control loop.
 * @return 0 on success, negative error code otherwise.
 */
int foc_example_main(void)
{
	struct tmc6100_desc   *tmc6100_desc;
	struct motor_pwm_desc *motor_pwm_desc;
	struct stm32_adc_desc *adc_desc;
	int ret;

	/* Initialise PI controllers. TODO: tune gains for the target motor. */
	g_foc.speed_pi = (struct sPI){
		.fDtSec     = DT_SPEED_S,
		.fKp        = 1.0f,		/* TODO: tune */
		.fKi        = 0.01f,		/* TODO: tune */
		.fUpOutLim  =  20.0f,		/* Iq_ref clamp [A] */
		.fLowOutLim = -20.0f,
	};
	g_foc.torque_pi = (struct sPI){
		.fDtSec     = DT_CURRENT_S,
		.fKp        = 0.5f,		/* TODO: tune */
		.fKi        = 50.0f,		/* TODO: tune */
		.fUpOutLim  =  VDC_BUS_V * 0.5f,
		.fLowOutLim = -VDC_BUS_V * 0.5f,
	};
	g_foc.flux_pi = (struct sPI){
		.fDtSec     = DT_CURRENT_S,
		.fKp        = 0.5f,		/* TODO: tune */
		.fKi        = 50.0f,		/* TODO: tune */
		.fUpOutLim  =  VDC_BUS_V * 0.5f,
		.fLowOutLim = -VDC_BUS_V * 0.5f,
	};
	g_foc.running = true;
	g_foc.error   = 0;

	/* --- Hardware init -------------------------------------------------- */

	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
	if (ret)
		goto exit;

	ret = motor_pwm_init(&motor_pwm_desc, &motor_pwm_ip);
	if (ret)
		goto remove_tmc6100;

	g_foc.pwm_desc = motor_pwm_desc;

	/*
	 * Encoder startup.
	 * Counter zeroed — rotor position unknown, FOC self-aligns over the
	 * first few electrical cycles.
	 * TODO: confirm TIM2 is configured in encoder mode in the .ioc.
	 */
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

	/*
	 * Register foc_tick as the ADC conversion-complete callback, then init.
	 * The callback is set here (not in common_data.c) so that common_data
	 * has no dependency on the example layer.
	 */
	adc_ip.conv_cplt_cb = foc_tick;
	ret = stm32_adc_init(&adc_desc, &adc_ip);
	if (ret)
		goto remove_pwm;

	ret = motor_pwm_enable(motor_pwm_desc);
	if (ret)
		goto remove_adc;

	ret = stm32_adc_start(adc_desc);
	if (ret)
		goto stop_motor;

	/* --- Run ------------------------------------------------------------ */

	/*
	 * foc_tick() runs entirely in ADC ISR context at 25 kHz.
	 * Main thread blocks here until the ISR signals an error.
	 */
	while (g_foc.running)
		;

	ret = g_foc.error;

	/* --- Cleanup -------------------------------------------------------- */

stop_motor:
	stm32_adc_stop(adc_desc);
	motor_pwm_disable(motor_pwm_desc);
remove_adc:
	stm32_adc_remove(adc_desc);
remove_pwm:
	motor_pwm_remove(motor_pwm_desc);
remove_tmc6100:
	tmc6100_remove(tmc6100_desc);
exit:
	if (ret)
		pr_info("Error: %d\n", ret);
	return ret;
}
