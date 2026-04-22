/***************************************************************************//**
 *   @file   speed_estimator.c
 *   @brief  Angular velocity estimator — inter-edge timing implementation.
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
#include "speed_estimator.h"
#include "no_os_alloc.h"
#include "no_os_error.h"
#include <math.h>

/*
 * Each electrical revolution produces 6 Hall edges (one per 60° sector).
 * The electrical angular step per edge is therefore π/3 rad.
 */
#define ELEC_ANGLE_PER_EDGE_RAD  (M_PI / 3.0f)

int speed_estimator_init(struct speed_estimator **desc,
			 uint8_t pole_pairs, uint32_t timer_freq_hz)
{
	struct speed_estimator *s;

	if (!desc || pole_pairs == 0 || timer_freq_hz == 0)
		return -EINVAL;

	s = no_os_calloc(1, sizeof(*s));
	if (!s)
		return -ENOMEM;

	s->pole_pairs = pole_pairs;
	s->timer_freq_hz = timer_freq_hz;

	*desc = s;
	return 0;
}

int speed_estimator_remove(struct speed_estimator *desc)
{
	if (!desc)
		return -EINVAL;

	no_os_free(desc);
	return 0;
}

int speed_estimator_update(struct speed_estimator *desc, uint32_t current_tick)
{
	uint32_t delta;

	if (!desc)
		return -EINVAL;

	/* Handle tick counter wrap-around. */
	if (current_tick >= desc->last_edge_tick)
		delta = current_tick - desc->last_edge_tick;
	else
		delta = (UINT32_MAX - desc->last_edge_tick) + current_tick + 1;

	desc->last_edge_tick = current_tick;

	if (delta == 0)
		return 0;

	desc->period_ticks = delta;

	/*
	 * ω_elec [rad/s] = (π/3) / (delta / timer_freq_hz)
	 *                = (π/3) * timer_freq_hz / delta
	 *
	 * TODO: replace HAL_GetTick()-based timing (1 kHz, 1 ms resolution)
	 *       with timer input-capture for better low-speed accuracy.
	 */
	desc->omega_rad_s = ELEC_ANGLE_PER_EDGE_RAD *
			    (float)desc->timer_freq_hz / (float)delta;

	return 0;
}

float speed_estimator_get_omega(struct speed_estimator *desc)
{
	return desc ? desc->omega_rad_s : 0.0f;
}
