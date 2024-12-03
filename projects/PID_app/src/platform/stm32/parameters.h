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
#define UART_BAUDRATE			115200

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
#define PWM_GPIO_EXTRA			&pid_pwm_gpio_extra

#define PWM_SECTOR_ID			1
#define PWM_SECTOR_PERIOD_NS		20000
#define PWM_OPS				&stm32_pwm_ops

#define PWM_UH_EXTRA			&pid_pwm_uh_extra
#define PWM_VH_EXTRA			&pid_pwm_vh_extra
#define PWM_WH_EXTRA			&pid_pwm_wh_extra

#define PWM_UL_EXTRA			&pid_pwm_ul_extra
#define PWM_VL_EXTRA			&pid_pwm_vl_extra
#define PWM_WL_EXTRA			&pid_pwm_wl_extra

extern struct stm32_uart_init_param pid_uart_extra;
extern struct stm32_gpio_init_param pid_gpio_extra;
extern struct stm32_gpio_init_param pid_pwm_gpio_extra;
extern struct stm32_spi_init_param pid_spi_extra_tmc4671;
extern struct stm32_spi_init_param pid_spi_extra_tmc6100;
extern struct stm32_pwm_init_param pid_pwm_uh_extra;
extern struct stm32_pwm_init_param pid_pwm_vh_extra;
extern struct stm32_pwm_init_param pid_pwm_wh_extra;
extern struct stm32_pwm_init_param pid_pwm_ul_extra;
extern struct stm32_pwm_init_param pid_pwm_vl_extra;
extern struct stm32_pwm_init_param pid_pwm_wl_extra;

#endif /* __PARAMETERS_H__ */
