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
#include "motor_pwm.h"
#include "stm32_adc.h"

/* UART */
#define UART_IRQ_ID			USART3_IRQn
#define UART_DEVICE_ID			1
#define UART_BAUDRATE			1000000
#define UART_EXTRA			&bldc_uart_extra
#define UART_OPS			&stm32_uart_ops
extern UART_HandleTypeDef		huart3;
#define UART_INSTANCE			(&huart3)

/* SPI (TMC6100 only) */
#define SPI_OPS				&stm32_spi_ops
#define SPI_EXTRA_TMC6100		&bldc_spi_extra_tmc6100
#define SPI_CS_PORT_TMC6100		1
#define TMC6100_SPI_DEVICE_ID		1
#define TMC6100_SPI_BAUDRATE		100000
#define TMC6100_SPI_CS			2

/* TMC6100 gate driver enable GPIO */
#define TMC6100_GPIO_DRVEN_PORT_NUM	2
#define TMC6100_GPIO_DRVEN_PIN_NUM	8

/* GPIO */
#define GPIO_OPS			&stm32_gpio_ops
#define GPIO_EXTRA			&bldc_gpio_extra

/* Motor PWM — TIM1, center-aligned, 3 complementary pairs, 25 kHz.
 * MOTOR_PWM_PERIOD_TICKS = 216 MHz / (2 * 25 kHz) = 4320 (ARR value).
 * Used by SVPWM to produce integer CCR values directly. */
extern TIM_HandleTypeDef		htim1;
#define MOTOR_PWM_TIMER			(&htim1)
#define MOTOR_PWM_PERIOD_NS		40000		/* 25 kHz switching */
#define MOTOR_PWM_PERIOD_TICKS		4320U		/* ARR: 216 MHz / (2*25 kHz) */
#define MOTOR_PWM_DEADTIME_NS		200		/* 200 ns dead-time */
#define MOTOR_PWM_TRGO_EN		true		/* trigger ADC at period center */

/* Encoder — TIM2 in encoder mode (x4 quadrature, 32-bit counter).
 * TODO: configure htim2 in the .ioc after hardware schematic is finalised. */
extern TIM_HandleTypeDef		htim2;
#define MOTOR_POLE_PAIRS		4

/*
 * ADC — phase current sensing, injected channels triggered by TIM1 TRGO.
 *
 * Clock: APB2 (108 MHz) / ADC_CLOCK_SYNC_PCLK_DIV4 = 27 MHz < 36 MHz limit.
 * Iu and Iw are sampled from the TMC6100 eval board AD8418 amplifiers whose
 * output is centred at VDDA/2 at zero current (bipolar topology). Iv is
 * derived in software from KCL: Iv = -(Iu + Iw).
 *
 * Channel and pin numbers are placeholders — update from .ioc once the
 * hardware schematic is finalised.
 */
extern ADC_HandleTypeDef		hadc1;
#define MOTOR_ADC_HANDLE		(&hadc1)
#define MOTOR_ADC_IU_CHANNEL		ADC_CHANNEL_3	/* TODO: verify from .ioc */
#define MOTOR_ADC_IW_CHANNEL		ADC_CHANNEL_10	/* TODO: verify from .ioc */
#define MOTOR_ADC_SAMPLING_TIME		ADC_SAMPLETIME_3CYCLES
#define MOTOR_ADC_TRIGGER		ADC_EXTERNALTRIGINJECCONV_T1_TRGO
#define MOTOR_ADC_TRIGGER_EDGE		ADC_EXTERNALTRIGINJECCONVEDGE_RISING

/*
 * Current conversion: I [A] = (raw_count - MIDPOINT) * CURRENT_SCALE
 *
 * CURRENT_SCALE = VREF_MV / (RESOLUTION * SHUNT_OHM * AMP_GAIN * 1000)
 *
 * Example values below are placeholders — replace with actual hardware specs.
 * TODO: set MOTOR_SHUNT_RESISTANCE and MOTOR_CURRENT_GAIN from schematic.
 */
#define MOTOR_SHUNT_RESISTANCE		0.01f		/* shunt [Ω]  — TODO */
#define MOTOR_CURRENT_GAIN		20.0f		/* amplifier gain — TODO */
#define MOTOR_ADC_VREF_MV		3300.0f		/* VDDA [mV] */
#define MOTOR_ADC_RESOLUTION		4096.0f		/* 12-bit, 2^12 counts */
#define MOTOR_ADC_MIDPOINT		2048.0f		/* counts at zero current */
#define MOTOR_ADC_CURRENT_SCALE \
	(MOTOR_ADC_VREF_MV / \
	 (MOTOR_ADC_RESOLUTION * MOTOR_SHUNT_RESISTANCE * \
	  MOTOR_CURRENT_GAIN * 1000.0f))

extern struct stm32_uart_init_param	bldc_uart_extra;
extern struct stm32_gpio_init_param	bldc_gpio_extra;
extern struct stm32_spi_init_param	bldc_spi_extra_tmc6100;

#endif /* __PARAMETERS_H__ */
