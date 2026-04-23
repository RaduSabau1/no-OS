/***************************************************************************//**
 *   @file   stm32/stm32_adc.h
 *   @brief  STM32 ADC driver for injected-channel synchronised current sampling.
 *
 *           Configures one ADC instance (ADC1 or ADC2) to sample up to four
 *           injected channels triggered by an external event (typically TIM1
 *           TRGO). All channels convert sequentially within a single injection
 *           sequence; the conversion-complete callback fires once per trigger
 *           after all channels have been sampled.
 *
 *           Intended use: FOC phase-current sensing synchronised to the centre
 *           of the PWM period (carrier valley at CNT = 0).
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
#ifndef STM32_ADC_H_
#define STM32_ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32_hal.h"

/* Forward declaration needed for the callback signature in stm32_adc_init_param. */
struct stm32_adc_desc;

/**
 * @struct stm32_adc_channel
 * @brief  Configuration for a single injected ADC channel.
 */
struct stm32_adc_channel {
	/** ADC channel identifier (ADC_CHANNEL_x HAL constant). */
	uint32_t channel;
	/** Injection sequence rank (ADC_INJECTED_RANK_1 .. ADC_INJECTED_RANK_4). */
	uint32_t rank;
	/** Sampling time (ADC_SAMPLETIME_x HAL constant). */
	uint32_t sampling_time;
	/** Digital offset subtracted from the raw result before it is stored.
	 *  Set to 0 when no hardware offset is applied. */
	uint32_t offset;
};

/**
 * @struct stm32_adc_init_param
 * @brief  Initialisation parameters for the STM32 ADC injected-channel driver.
 */
struct stm32_adc_init_param {
	/** Pointer to the ADC HAL handle (ADC_HandleTypeDef*). */
	void *hadc;
	/** Number of injected channels to configure (1–4). */
	uint8_t num_channels;
	/** Array of per-channel configuration, length must be num_channels. */
	struct stm32_adc_channel *channels;
	/** External trigger source (ADC_EXTERNALTRIGINJECCONV_x). */
	uint32_t ext_trigger;
	/** External trigger active edge (ADC_EXTERNALTRIGINJECCONVEDGE_x). */
	uint32_t ext_trigger_edge;
	/**
	 * Callback invoked from HAL_ADCEx_InjectedConvCpltCallback after all
	 * injected channels in one sequence have been converted.
	 * Runs in interrupt context — keep execution time short.
	 * Set to NULL for polling use only.
	 */
	void (*conv_cplt_cb)(struct stm32_adc_desc *desc);
};

/**
 * @struct stm32_adc_desc
 * @brief  Runtime descriptor returned by stm32_adc_init().
 */
struct stm32_adc_desc {
	/** ADC HAL handle in use. */
	void    *hadc;
	/** Number of injected channels configured. */
	uint8_t  num_channels;
	/** Conversion-complete callback (may be NULL). */
	void   (*conv_cplt_cb)(struct stm32_adc_desc *desc);
};

/**
 * @brief Initialise the ADC for synchronised injected-channel operation.
 *
 * Configures the ADC base (12-bit, right-aligned, scan enabled) and each
 * injected channel with the supplied trigger source. If conv_cplt_cb is
 * non-NULL, the ISR dispatch is registered and interrupts are used; otherwise
 * the driver operates in polling mode.
 *
 * @param desc   Output descriptor pointer.
 * @param param  Initialisation parameters.
 * @return 0 on success, negative error code otherwise.
 */
int stm32_adc_init(struct stm32_adc_desc **desc,
		   struct stm32_adc_init_param *param);

/**
 * @brief Stop conversions, deinitialise the ADC, and free the descriptor.
 * @param desc  Descriptor to free.
 * @return 0 on success, negative error code otherwise.
 */
int stm32_adc_remove(struct stm32_adc_desc *desc);

/**
 * @brief Start injected conversions.
 *
 * Uses interrupt mode when conv_cplt_cb was supplied at init, polling
 * mode otherwise.
 *
 * @param desc  Driver descriptor.
 * @return 0 on success, negative error code otherwise.
 */
int stm32_adc_start(struct stm32_adc_desc *desc);

/**
 * @brief Stop injected conversions.
 * @param desc  Driver descriptor.
 * @return 0 on success, negative error code otherwise.
 */
int stm32_adc_stop(struct stm32_adc_desc *desc);

/**
 * @brief Read the last converted value for a given injected rank.
 *
 * Valid after a conversion completes (either from the callback or after
 * polling). The result is a raw 12-bit right-aligned count [0, 4095].
 *
 * @param desc  Driver descriptor.
 * @param rank  Injected rank (ADC_INJECTED_RANK_1 .. ADC_INJECTED_RANK_4).
 * @return Raw ADC count, or 0 on error.
 */
uint32_t stm32_adc_get_value(struct stm32_adc_desc *desc, uint32_t rank);

#endif /* STM32_ADC_H_ */
