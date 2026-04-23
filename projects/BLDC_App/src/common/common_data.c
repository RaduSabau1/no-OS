/***************************************************************************//**
 *   @file   common_data.c
 *   @brief  Defines common data and initialization parameters for BLDC_App.
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
#include "common_data.h"

struct no_os_uart_init_param bldc_app_uart_ip = {
	.device_id = UART_DEVICE_ID,
	.baud_rate = UART_BAUDRATE,
	.size = NO_OS_UART_CS_8,
	.platform_ops = UART_OPS,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.extra = UART_EXTRA,
};

struct no_os_spi_init_param tmc6100_comm_param = {
	.device_id = TMC6100_SPI_DEVICE_ID,
	.extra = SPI_EXTRA_TMC6100,
	.max_speed_hz = TMC6100_SPI_BAUDRATE,
	.platform_ops = SPI_OPS,
	.chip_select = TMC6100_SPI_CS,
	.mode = NO_OS_SPI_MODE_3,
};

struct no_os_gpio_init_param tmc6100_drv_en_param = {
	.port = TMC6100_GPIO_DRVEN_PORT_NUM,
	.pull = NO_OS_PULL_NONE,
	.number = TMC6100_GPIO_DRVEN_PIN_NUM,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA,
};

struct tmc6100_init_param tmc6100_ip = {
	.comm_param = &tmc6100_comm_param,
	.drv_en_param = &tmc6100_drv_en_param,
};

struct motor_pwm_init_param motor_pwm_ip = {
	.htimer = MOTOR_PWM_TIMER,
	.period_ns = MOTOR_PWM_PERIOD_NS,
	.deadtime_ns = MOTOR_PWM_DEADTIME_NS,
	.trgo_enable = MOTOR_PWM_TRGO_EN,
	.get_timer_clock = HAL_RCC_GetHCLKFreq,
};

/*
 * Injected channel configuration for Iu (rank 1) and Iw (rank 2).
 * Channel numbers are placeholders — update MOTOR_ADC_IU_CHANNEL and
 * MOTOR_ADC_IW_CHANNEL in parameters.h after .ioc pin assignment.
 * conv_cplt_cb is set to foc_tick() by foc_example_main() before calling
 * stm32_adc_init(), keeping common_data free of example-layer dependencies.
 */
static struct stm32_adc_channel motor_adc_channels[] = {
	{
		.channel      = MOTOR_ADC_IU_CHANNEL,
		.rank         = ADC_INJECTED_RANK_1,
		.sampling_time = MOTOR_ADC_SAMPLING_TIME,
		.offset       = 0,
	},
	{
		.channel      = MOTOR_ADC_IW_CHANNEL,
		.rank         = ADC_INJECTED_RANK_2,
		.sampling_time = MOTOR_ADC_SAMPLING_TIME,
		.offset       = 0,
	},
};

struct stm32_adc_init_param adc_ip = {
	.hadc             = MOTOR_ADC_HANDLE,
	.num_channels     = NO_OS_ARRAY_SIZE(motor_adc_channels),
	.channels         = motor_adc_channels,
	.ext_trigger      = MOTOR_ADC_TRIGGER,
	.ext_trigger_edge = MOTOR_ADC_TRIGGER_EDGE,
	.conv_cplt_cb     = NULL, /* set to foc_tick() in foc_example_main() */
};

