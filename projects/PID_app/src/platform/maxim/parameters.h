#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include "maxim_gpio.h"
#include "maxim_spi.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"
#include "maxim_pwm.h"

#define	UART_IRQ_ID			UART0_IRQn
#define UART_DEVICE_ID			0
#define UART_BAUDRATE			57600

#define UART_EXTRA			&pid_uart_extra
#define UART_OPS			&max_uart_ops

#define SPI_OPS				&max_spi_ops
#define SPI_EXTRA			&pid_spi_extra

#define TMC4671_SPI_DEVICE_ID		4
#define TMC4671_SPI_BAUDRATE		100000
#define TMC4671_SPI_CS			0

#define TMC4671_GPIO_RSTN_PORT_NUM	2
#define TMC4671_GPIO_RSTN_PIN_NUM	21

#define TMC6100_SPI_DEVICE_ID		1
#define TMC6100_SPI_BAUDRATE		100000
#define TMC6100_SPI_CS			0

#define TMC6100_GPIO_DRVEN_PORT_NUM	2
#define TMC6100_GPIO_DRVEN_PIN_NUM	21

#define TMC6100_GPIO_SPE_PORT_NUM	2
#define TMC6100_GPIO_SPE_PIN_NUM	31

#define TMC6100_GPIO_UH_PORT_NUM	1
#define TMC6100_GPIO_UH_PIN_NUM		24

#define TMC6100_GPIO_UL_PORT_NUM	1
#define TMC6100_GPIO_UL_PIN_NUM		25

#define TMC6100_GPIO_VH_PORT_NUM	2
#define TMC6100_GPIO_VH_PIN_NUM		13

#define TMC6100_GPIO_VL_PORT_NUM	2
#define TMC6100_GPIO_VL_PIN_NUM		15

#define TMC6100_GPIO_WH_PORT_NUM	2
#define TMC6100_GPIO_WH_PIN_NUM		14

#define TMC6100_GPIO_WL_PORT_NUM	2
#define TMC6100_GPIO_WL_PIN_NUM		16





#define TMC6100_UVW_ENABLE_PORT_NUM	2
#define TMC6100_UVW_ENABLE_PIN_NUM	6

#define TMC6100_U_PWM_ID		1
#define TMC6100_V_PWM_ID		2
#define TMC6100_W_PWM_ID		4



#define PID_GPIO_PWMIN_PORT_NUM		2
#define PID_GPIO_PWMIN_PIN_NUM		7

#define GPIO_OPS			&max_gpio_ops
#define GPIO_EXTRA			&pid_gpio_extra

#define PWM_SECTOR_ID			1
#define PWM_SECTOR_PERIOD_NS		40000
#define PWM_OPS				&max_pwm_ops
#define PWM_EXTRA			&pid_pwm_extra

extern struct max_uart_init_param pid_uart_extra;
extern struct max_gpio_init_param pid_gpio_extra;
extern struct max_spi_init_param pid_spi_extra;
extern struct max_pwm_init_param pid_pwm_extra;

#endif /* __PARAMETERS_H__ */
