#include "parameters.h"

struct max_uart_init_param pid_uart_extra = {
	.flow = UART_FLOW_DIS,
};

struct max_gpio_init_param pid_gpio_extra = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct max_spi_init_param pid_spi_extra = {
	.num_slaves = 1,
	.polarity = SPI_SS_POL_LOW,
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};

struct max_pwm_init_param pid_pwm_extra = {
	.vssel = MXC_GPIO_VSSEL_VDDIOH,
};
