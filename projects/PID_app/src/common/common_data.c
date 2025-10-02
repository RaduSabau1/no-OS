/***************************************************************************//**
 *   @file   common_data.c
 *   @brief  Defines common data and initialization parameters for PID_App.
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

struct no_os_uart_init_param pid_app_uart_ip = {
	.device_id = UART_DEVICE_ID,
	.baud_rate = UART_BAUDRATE,
	.size = NO_OS_UART_CS_8,
	.platform_ops = UART_OPS,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.extra = UART_EXTRA,
};

struct no_os_spi_init_param tmc4671_spi_param = {
	.device_id = TMC4671_SPI_DEVICE_ID,
	.extra = SPI_EXTRA_TMC4671,
	.max_speed_hz = TMC4671_SPI_BAUDRATE,
	.platform_ops = SPI_OPS,
	.chip_select = TMC4671_SPI_CS,
	.mode = NO_OS_SPI_MODE_3,
};

struct tmc4671_init_param tmc4671_ip = {
	.spi_param = &tmc4671_spi_param,
	.motor_type = TMC4671_THREE_PHASE_BLDC,
	.no_pole_pairs = 4,
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
	.extra = GPIO_EXTRA
};

struct tmc6100_init_param tmc6100_ip = {
	.comm_param = &tmc6100_comm_param,
	.drv_en_param = &tmc6100_drv_en_param,
	.pwm_period_ns = 20000,
	.ext_ctrl = true,
};