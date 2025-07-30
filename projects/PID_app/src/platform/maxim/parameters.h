#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include "maxim_irq.h"
#include "maxim_gpio.h"
#include "maxim_spi.h"
#include "maxim_uart.h"
#include "maxim_uart_stdio.h"

#define	UART_IRQ_ID			UART0_IRQn
#define UART_DEVICE_ID			0
#define UART_BAUDRATE			1000000

#define UART_EXTRA			&max_uart_extra
#define UART_OPS			&max_uart_ops

#define SPI_OPS				&max_spi_ops
#define SPI_EXTRA_TMC4671		&max_spi_extra
#define SPI_EXTRA_TMC6100		&max_spi_extra

#define TMC4671_SPI_DEVICE_ID		4
#define TMC4671_SPI_BAUDRATE		10000000
#define TMC4671_SPI_CS			0

#define TMC6100_SPI_DEVICE_ID		1
#define TMC6100_SPI_BAUDRATE		100000
#define TMC6100_SPI_CS			0

#define TMC6100_GPIO_DRVEN_PORT_NUM	2
#define TMC6100_GPIO_DRVEN_PIN_NUM	21

#define GPIO_OPS			&max_gpio_ops
#define GPIO_EXTRA			&max_gpio_extra

extern struct max_uart_init_param max_uart_extra;
extern struct max_gpio_init_param max_gpio_extra;
extern struct max_spi_init_param max_spi_extra;

#endif /* __PARAMETERS_H__ */
