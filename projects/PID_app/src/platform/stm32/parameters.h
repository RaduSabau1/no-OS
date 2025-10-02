/***************************************************************************//**
 *   @file   parameters.h
 *   @brief  Platform-specific parameter defines for STM32 platform.
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
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include "stm32_hal.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include "stm32_uart.h"
#include "stm32_uart_stdio.h"
#include "stm32_pwm.h"

#define	UART_IRQ_ID			USART3_IRQn
#define UART_DEVICE_ID			1
#define UART_BAUDRATE			1000000

#define UART_EXTRA			&pid_uart_extra
#define UART_OPS			&stm32_uart_ops
extern UART_HandleTypeDef		huart3;
#define UART_INSTANCE			(&huart3)

#define SPI_OPS				&stm32_spi_ops
#define SPI_EXTRA_TMC4671		&pid_spi_extra_tmc4671
#define SPI_EXTRA_TMC6100		&pid_spi_extra_tmc6100

#define SPI_CS_PORT_TMC4671		0
#define SPI_CS_PORT_TMC6100		1

#define TMC4671_SPI_DEVICE_ID		2
#define TMC4671_SPI_BAUDRATE		40000000
#define TMC4671_SPI_CS			4

#define TMC6100_SPI_DEVICE_ID		1
#define TMC6100_SPI_BAUDRATE		100000
#define TMC6100_SPI_CS			2

#define TMC6100_GPIO_DRVEN_PORT_NUM	2
#define TMC6100_GPIO_DRVEN_PIN_NUM	8

#define TMC6100_PWMH_ID			1
#define TMC6100_PWML_ID			1

#define GPIO_OPS			&stm32_gpio_ops
#define GPIO_EXTRA			&pid_gpio_extra

#define PWM_SECTOR_ID			1
#define PWM_SECTOR_PERIOD_NS		20000
#define PWM_OPS				&stm32_pwm_ops

extern struct stm32_uart_init_param pid_uart_extra;
extern struct stm32_gpio_init_param pid_gpio_extra;
extern struct stm32_spi_init_param pid_spi_extra_tmc4671;
extern struct stm32_spi_init_param pid_spi_extra_tmc6100;

#endif /* __PARAMETERS_H__ */
