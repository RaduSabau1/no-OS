/***************************************************************************//**
 *   @file   stm32/stm32_adc.c
 *   @brief  STM32 ADC driver for injected-channel synchronised current sampling.
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
#include "stm32_adc.h"
#include "no_os_alloc.h"
#include "no_os_error.h"

/*
 * Single global descriptor used for ISR dispatch.
 * Only one ADC instance is needed for the FOC current sensing path.
 */
static struct stm32_adc_desc *g_adc_desc;

/**
 * HAL weak-symbol override.  Called by the HAL interrupt handler after all
 * injected channels in one sequence have been converted.  Dispatches to the
 * user callback stored in the descriptor.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (g_adc_desc && hadc == g_adc_desc->hadc && g_adc_desc->conv_cplt_cb)
		g_adc_desc->conv_cplt_cb(g_adc_desc);
}

int stm32_adc_init(struct stm32_adc_desc **desc,
		   struct stm32_adc_init_param *param)
{
	ADC_InjectionConfTypeDef inj_config = {0};
	ADC_HandleTypeDef *hadc;
	struct stm32_adc_desc *d;
	int ret;
	uint8_t i;

	if (!desc || !param || !param->hadc || !param->channels ||
	    param->num_channels == 0 || param->num_channels > 4)
		return -EINVAL;

	d = no_os_calloc(1, sizeof(*d));
	if (!d)
		return -ENOMEM;

	hadc = (ADC_HandleTypeDef *)param->hadc;

	/*
	 * ADC base configuration.
	 *   - 12-bit resolution, right-aligned: gives counts in [0, 4095].
	 *   - Scan enabled: required when more than one injected channel is used.
	 *   - Continuous and regular-channel DMA disabled: injected path only.
	 *   - Clock: APB2 (108 MHz on STM32F7) / DIV4 = 27 MHz < 36 MHz limit.
	 */
	hadc->Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc->Init.Resolution            = ADC_RESOLUTION_12B;
	hadc->Init.ScanConvMode          = ENABLE;
	hadc->Init.ContinuousConvMode    = DISABLE;
	hadc->Init.DiscontinuousConvMode = DISABLE;
	hadc->Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc->Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	hadc->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	hadc->Init.NbrOfConversion       = 1;
	hadc->Init.DMAContinuousRequests = DISABLE;
	hadc->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(hadc) != HAL_OK) {
		ret = -EIO;
		goto free_desc;
	}

	/*
	 * Injected channel configuration.
	 * InjectedNbrOfConversion and trigger settings must be written for every
	 * rank — HAL applies them from the struct on each call.
	 * AutoInjectedConv disabled: external trigger drives the sequence.
	 */
	inj_config.InjectedNbrOfConversion       = param->num_channels;
	inj_config.ExternalTrigInjecConv         = param->ext_trigger;
	inj_config.ExternalTrigInjecConvEdge     = param->ext_trigger_edge;
	inj_config.AutoInjectedConv              = DISABLE;
	inj_config.InjectedDiscontinuousConvMode = DISABLE;

	for (i = 0; i < param->num_channels; i++) {
		inj_config.InjectedChannel      = param->channels[i].channel;
		inj_config.InjectedRank         = param->channels[i].rank;
		inj_config.InjectedSamplingTime = param->channels[i].sampling_time;
		inj_config.InjectedOffset       = param->channels[i].offset;
		if (HAL_ADCEx_InjectedConfigChannel(hadc, &inj_config) != HAL_OK) {
			ret = -EIO;
			goto deinit_adc;
		}
	}

	d->hadc          = param->hadc;
	d->num_channels  = param->num_channels;
	d->conv_cplt_cb  = param->conv_cplt_cb;

	/* Register global descriptor for ISR dispatch. */
	g_adc_desc = d;

	*desc = d;
	return 0;

deinit_adc:
	HAL_ADC_DeInit(hadc);
free_desc:
	no_os_free(d);
	return ret;
}

int stm32_adc_remove(struct stm32_adc_desc *desc)
{
	if (!desc)
		return -EINVAL;

	if (g_adc_desc == desc)
		g_adc_desc = NULL;

	HAL_ADC_DeInit((ADC_HandleTypeDef *)desc->hadc);
	no_os_free(desc);
	return 0;
}

int stm32_adc_start(struct stm32_adc_desc *desc)
{
	ADC_HandleTypeDef *hadc;

	if (!desc || !desc->hadc)
		return -EINVAL;

	hadc = (ADC_HandleTypeDef *)desc->hadc;

	if (desc->conv_cplt_cb) {
		/* Interrupt mode: ISR fires after each injection sequence. */
		if (HAL_ADCEx_InjectedStart_IT(hadc) != HAL_OK)
			return -EIO;
	} else {
		/* Polling mode: caller reads values after waiting for JEOC. */
		if (HAL_ADCEx_InjectedStart(hadc) != HAL_OK)
			return -EIO;
	}

	return 0;
}

int stm32_adc_stop(struct stm32_adc_desc *desc)
{
	ADC_HandleTypeDef *hadc;

	if (!desc || !desc->hadc)
		return -EINVAL;

	hadc = (ADC_HandleTypeDef *)desc->hadc;

	if (desc->conv_cplt_cb)
		HAL_ADCEx_InjectedStop_IT(hadc);
	else
		HAL_ADCEx_InjectedStop(hadc);

	return 0;
}

uint32_t stm32_adc_get_value(struct stm32_adc_desc *desc, uint32_t rank)
{
	if (!desc || !desc->hadc)
		return 0;

	return HAL_ADCEx_InjectedGetValue((ADC_HandleTypeDef *)desc->hadc, rank);
}
