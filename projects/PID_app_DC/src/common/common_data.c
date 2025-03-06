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
	.motor_type = TMC4671_SINGLE_PHASE_DC,
	.no_pole_pairs = 2,
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
	.pwm_period_ns = PWM_SECTOR_PERIOD_NS,
	.ext_ctrl = true,
};