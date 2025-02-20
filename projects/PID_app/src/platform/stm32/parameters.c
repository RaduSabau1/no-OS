#include "parameters.h"

struct stm32_uart_init_param pid_uart_extra = {
	.huart = UART_INSTANCE,
};

struct stm32_gpio_init_param pid_gpio_extra = {
	.mode = GPIO_MODE_OUTPUT_PP,
	.speed = GPIO_SPEED_HIGH,
};

struct stm32_spi_init_param pid_spi_extra_tmc4671 = {
	.chip_select_port = SPI_CS_PORT_TMC4671,
	.get_input_clock = HAL_RCC_GetHCLKFreq,
};

struct stm32_spi_init_param pid_spi_extra_tmc6100 = {
	.chip_select_port = SPI_CS_PORT_TMC6100,
	.get_input_clock = HAL_RCC_GetHCLKFreq,
};
