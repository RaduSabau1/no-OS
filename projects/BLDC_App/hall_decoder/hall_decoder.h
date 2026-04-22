/***************************************************************************//**
 *   @file   hall_decoder.h
 *   @brief  Hall sensor state decoder — maps 3-bit Hall state to electrical
 *           angle for a 6-step BLDC motor.
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
#ifndef __HALL_DECODER_H__
#define __HALL_DECODER_H__

#include <stdint.h>

/**
 * @brief Hall decoder descriptor.
 *
 * Hall state encoding: bit2 = H3, bit1 = H2, bit0 = H1.
 * Valid states: 1–6 (0 and 7 indicate wiring fault / all-low / all-high).
 * Electrical angle resolution: 60° per step (6 steps per electrical revolution).
 */
struct hall_decoder {
	uint8_t state;		  /* current Hall state (0–7) */
	uint8_t prev_state;	  /* previous Hall state */
	float elec_angle_rad;	  /* electrical angle [0, 2π] */
};

/**
 * @brief Allocate and initialise a Hall decoder instance.
 * @param desc  Output descriptor pointer.
 * @return 0 on success, negative error code otherwise.
 */
int hall_decoder_init(struct hall_decoder **desc);

/**
 * @brief Free a Hall decoder instance.
 * @param desc  Descriptor to free.
 * @return 0 on success, negative error code otherwise.
 */
int hall_decoder_remove(struct hall_decoder *desc);

/**
 * @brief Update the decoder with fresh Hall sensor readings.
 * @param desc  Decoder descriptor.
 * @param h1    Logic level of Hall sensor H1 (0 or 1).
 * @param h2    Logic level of Hall sensor H2 (0 or 1).
 * @param h3    Logic level of Hall sensor H3 (0 or 1).
 * @return 0 on success, -EINVAL if state is invalid (0 or 7).
 */
int hall_decoder_update(struct hall_decoder *desc,
			uint8_t h1, uint8_t h2, uint8_t h3);

/**
 * @brief Return the current electrical angle in radians.
 * @param desc  Decoder descriptor.
 * @return Electrical angle [0, 2π].
 */
float hall_decoder_get_angle(struct hall_decoder *desc);

#endif /* __HALL_DECODER_H__ */
