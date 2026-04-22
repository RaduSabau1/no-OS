/***************************************************************************//**
 *   @file   speed_estimator.h
 *   @brief  Angular velocity estimator based on Hall sensor inter-edge timing.
 *
 *           Initial implementation uses HAL_GetTick() (1 ms resolution).
 *           A future upgrade to timer input-capture will improve low-speed
 *           accuracy; a closed-loop observer (PLL or Luenberger) may be
 *           added as a further research contribution if time permits.
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
#ifndef __SPEED_ESTIMATOR_H__
#define __SPEED_ESTIMATOR_H__

#include <stdint.h>

/**
 * @brief Speed estimator descriptor.
 *
 * Estimates ω [rad/s] from the time between consecutive Hall sensor edges.
 * One electrical revolution produces 6 edges; combined with pole_pairs the
 * mechanical speed is also derivable.
 */
struct speed_estimator {
	uint32_t last_edge_tick;  /* tick count at last Hall edge           */
	uint32_t period_ticks;	  /* measured period between edges [ticks]  */
	float omega_rad_s;	  /* estimated electrical angular velocity   */
	uint8_t pole_pairs;	  /* motor pole pairs                        */
	uint32_t timer_freq_hz;	  /* tick source frequency [Hz]              */
};

/**
 * @brief Allocate and initialise a speed estimator.
 * @param desc          Output descriptor pointer.
 * @param pole_pairs    Motor pole-pair count.
 * @param timer_freq_hz Tick source frequency in Hz (e.g. 1000 for HAL_GetTick).
 * @return 0 on success, negative error code otherwise.
 */
int speed_estimator_init(struct speed_estimator **desc,
			 uint8_t pole_pairs, uint32_t timer_freq_hz);

/**
 * @brief Free a speed estimator instance.
 * @param desc  Descriptor to free.
 * @return 0 on success, negative error code otherwise.
 */
int speed_estimator_remove(struct speed_estimator *desc);

/**
 * @brief Notify the estimator that a Hall edge just occurred.
 * @param desc         Estimator descriptor.
 * @param current_tick Current timer tick value.
 * @return 0 on success, negative error code otherwise.
 */
int speed_estimator_update(struct speed_estimator *desc, uint32_t current_tick);

/**
 * @brief Return the latest estimated electrical angular velocity.
 * @param desc  Estimator descriptor.
 * @return ω [rad/s].
 */
float speed_estimator_get_omega(struct speed_estimator *desc);

#endif /* __SPEED_ESTIMATOR_H__ */
