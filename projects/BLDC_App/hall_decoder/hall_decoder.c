/***************************************************************************//**
 *   @file   hall_decoder.c
 *   @brief  Hall sensor state decoder implementation.
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
#include "hall_decoder.h"
#include "no_os_alloc.h"
#include "no_os_error.h"
#include <math.h>

/*
 * Lookup table: Hall state (index 0–7) → electrical angle [rad].
 * States 0 and 7 are invalid (all sensors low / all high).
 * Angle assignments assume standard BLDC Hall wiring; adjust if motor
 * spins in the wrong direction or if Hall sensor order differs.
 *
 *  State | H3 H2 H1 | Sector | Angle (centre of 60° sector)
 *  ------|----------|--------|-----------------------------
 *    1   |  0  0  1 |   1    |  0°  (0 rad)
 *    2   |  0  1  0 |   3    | 120° (2π/3 rad)
 *    3   |  0  1  1 |   2    |  60° (π/3 rad)
 *    4   |  1  0  0 |   5    | 240° (4π/3 rad)
 *    5   |  1  0  1 |   6    | 300° (5π/3 rad)
 *    6   |  1  1  0 |   4    | 180° (π rad)
 */
static const float hall_angle_lut[8] = {
	0.0f,			/* 0 — invalid */
	0.0f,			/* 1 — 0° */
	2.0f * M_PI / 3.0f,	/* 2 — 120° */
	M_PI / 3.0f,		/* 3 — 60° */
	4.0f * M_PI / 3.0f,	/* 4 — 240° */
	5.0f * M_PI / 3.0f,	/* 5 — 300° */
	M_PI,			/* 6 — 180° */
	0.0f,			/* 7 — invalid */
};

int hall_decoder_init(struct hall_decoder **desc)
{
	struct hall_decoder *d;

	if (!desc)
		return -EINVAL;

	d = no_os_calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;

	*desc = d;
	return 0;
}

int hall_decoder_remove(struct hall_decoder *desc)
{
	if (!desc)
		return -EINVAL;

	no_os_free(desc);
	return 0;
}

int hall_decoder_update(struct hall_decoder *desc,
			uint8_t h1, uint8_t h2, uint8_t h3)
{
	uint8_t state;

	if (!desc)
		return -EINVAL;

	state = ((h3 & 1u) << 2) | ((h2 & 1u) << 1) | (h1 & 1u);

	if (state == 0 || state == 7)
		return -EINVAL;

	desc->prev_state = desc->state;
	desc->state = state;
	desc->elec_angle_rad = hall_angle_lut[state];

	return 0;
}

float hall_decoder_get_angle(struct hall_decoder *desc)
{
	return desc ? desc->elec_angle_rad : 0.0f;
}
