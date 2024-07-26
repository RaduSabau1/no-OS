#include "common_data.h"

struct no_os_uart_init_param accel_uart_ip = {
	.device_id = UART_DEVICE_ID,
	.irq_id = UART_IRQ_ID,
	.asynchronous_rx = true,
	.baud_rate = UART_BAUDRATE,
	.size = NO_OS_UART_CS_8,
	.parity = NO_OS_UART_PAR_NO,
	.stop = NO_OS_UART_STOP_1_BIT,
	.extra = UART_EXTRA,
	.platform_ops = UART_OPS,
};


struct no_os_spi_init_param ssd_1306_spi_init_param = {
	.device_id = SPI_DEVICE_D_ID,
	.max_speed_hz = SPI_BAUDRATE,
	.mode = NO_OS_SPI_MODE_0,
	.chip_select = SPI_D_CS,
	.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.platform_ops = SPI_OPS,
	.extra = SPI_EXTRA
};

struct no_os_gpio_init_param dc_pin = {
	.port = GPIO_DC_PORT,
	.number = GPIO_DC_PIN,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA
};
struct no_os_gpio_init_param reset_pin = {
	.port = GPIO_RESET_PORT,
	.number = GPIO_RESET_PIN,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA
};

struct no_os_gpio_init_param vbat_pin = {
	.port = GPIO_VBAT_PORT,
	.number = GPIO_VBAT_PIN,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA
};

struct no_os_gpio_init_param vdd_pin = {
	.port = GPIO_VDD_PORT,
	.number = GPIO_VDD_PIN,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA
};

struct ssd_1306_extra ssd1306_extra = {
	.dc_pin_ip = &dc_pin,
	.reset_pin_ip = &reset_pin,
	.spi_ip = &ssd_1306_spi_init_param
};

struct display_init_param display_ip = {
	.cols_nb = 16U,
	.rows_nb = 4U,
	.controller_ops = &ssd1306_ops,
	.extra = &ssd1306_extra
};

struct no_os_spi_init_param adxl355_spi_ip = {
	.device_id = SPI_DEVICE_A_ID,
	.max_speed_hz = SPI_BAUDRATE,
	.bit_order = NO_OS_SPI_BIT_ORDER_MSB_FIRST,
	.mode = NO_OS_SPI_MODE_0,
	.platform_ops = SPI_OPS,
	.chip_select = SPI_A_CS,
	.extra = SPI_EXTRA,
};

struct adxl355_init_param adxl355_ip = {
	.comm_type = ADXL355_SPI_COMM,
	.dev_type = ID_ADXL355,
};