/***************************************************************************//**
 *   @file   svpwm.c
 *   @brief  Space Vector PWM computation implementation.
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
#include "svpwm.h"
#include <math.h>

#define SQRT3       1.7320508075688772f
#define SQRT3_HALF  0.8660254037844386f

/*
 * Sector identification and duty cycle computation using the symmetrical
 * SVPWM (centre-aligned) method.
 *
 * Reference frame: standard 60°-sector decomposition of the αβ plane.
 *   Sector 1: 0° – 60°   (Vα > 0,  Vβ/√3 < Vα, Vβ ≥ 0)
 *   ...
 *
 * Duty cycles are computed as:
 *   tu = (1 + t1 - t2) / 2
 *   tv = (1 - t1 + t2) / 2   (varies by sector, see below)
 *   tw = (1 - t1 - t2) / 2
 *
 * where t1, t2 are the normalised active-vector times [0, 1].
 */
void svpwm_compute(float v_alpha, float v_beta, float v_dc,
		   uint16_t period_ticks, struct svpwm_out *out)
{
	float vref1, vref2, vref3;
	float t1, t2, t0;
	float pt;
	int sector;

	if (!out || v_dc == 0.0f)
		return;

	pt = (float)period_ticks;

	/* Normalise to [–1, 1] relative to Vdc/2 */
	float va = v_alpha / (v_dc * 0.5f);
	float vb = v_beta  / (v_dc * 0.5f);

	/* Reference voltages projected onto the three sector boundaries. */
	vref1 =  vb;
	vref2 =  vb * 0.5f + SQRT3_HALF * va;   /* = (√3·Vα + Vβ) / 2 */
	vref3 =  vb * 0.5f - SQRT3_HALF * va;   /* = (√3·Vα - Vβ) / 2 — negated */

	/* Determine sector (1–6). */
	if (vref1 > 0.0f) {
		if (vref2 > 0.0f) {
			sector = (vref3 > 0.0f) ? 2 : 1;
		} else {
			sector = 6;
		}
	} else {
		if (vref2 > 0.0f) {
			sector = 3;
		} else {
			sector = (vref3 > 0.0f) ? 4 : 5;
		}
	}

	/* Active vector times per sector. */
	switch (sector) {
	case 1:
		t1 =  vref2;
		t2 =  vref1;
		break;
	case 2:
		t1 = -vref3;
		t2 = -vref2;
		break;
	case 3:
		t1 =  vref1;
		t2 =  vref3;
		break;
	case 4:
		t1 = -vref2;
		t2 = -vref1;
		break;
	case 5:
		t1 =  vref3;
		t2 =  vref2;
		break;
	case 6:
	default:
		t1 = -vref1;
		t2 = -vref3;
		break;
	}

	/* Clamp to modulation limit. */
	if (t1 + t2 > 1.0f) {
		float scale = 1.0f / (t1 + t2);
		t1 *= scale;
		t2 *= scale;
	}

	t0 = (1.0f - t1 - t2) * 0.5f;

	/* CCR tick values per sector — multiply normalised duty by period_ticks. */
	switch (sector) {
	case 1:
		out->tu = (uint16_t)((t1 + t2 + t0) * pt);
		out->tv = (uint16_t)((t2 + t0)       * pt);
		out->tw = (uint16_t)(t0               * pt);
		break;
	case 2:
		out->tu = (uint16_t)((t1 + t0)       * pt);
		out->tv = (uint16_t)((t1 + t2 + t0)  * pt);
		out->tw = (uint16_t)(t0               * pt);
		break;
	case 3:
		out->tu = (uint16_t)(t0               * pt);
		out->tv = (uint16_t)((t1 + t2 + t0)  * pt);
		out->tw = (uint16_t)((t2 + t0)        * pt);
		break;
	case 4:
		out->tu = (uint16_t)(t0               * pt);
		out->tv = (uint16_t)((t1 + t0)        * pt);
		out->tw = (uint16_t)((t1 + t2 + t0)  * pt);
		break;
	case 5:
		out->tu = (uint16_t)((t2 + t0)        * pt);
		out->tv = (uint16_t)(t0               * pt);
		out->tw = (uint16_t)((t1 + t2 + t0)  * pt);
		break;
	case 6:
	default:
		out->tu = (uint16_t)((t1 + t2 + t0)  * pt);
		out->tv = (uint16_t)(t0               * pt);
		out->tw = (uint16_t)((t1 + t0)        * pt);
		break;
	}
}
