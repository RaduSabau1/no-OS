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

struct no_os_gpio_init_param tmc6100_uh_gpio_param = {
	.port = 4,
	.number = 9,
	.platform_ops = GPIO_OPS,
	.extra = PWM_GPIO_EXTRA
};

struct no_os_gpio_init_param tmc6100_vh_gpio_param = {
	.port = 4,
	.number = 8,
	.platform_ops = GPIO_OPS,
	.extra = PWM_GPIO_EXTRA
};

struct no_os_gpio_init_param tmc6100_wh_gpio_param = {
	.port = 4,
	.number = 11,
	.platform_ops = GPIO_OPS,
	.extra = PWM_GPIO_EXTRA
};

struct no_os_gpio_init_param tmc6100_ul_gpio_param = {
	.port = 4,
	.number = 10,
	.platform_ops = GPIO_OPS,
	.extra = PWM_GPIO_EXTRA
};

struct no_os_gpio_init_param tmc6100_vl_gpio_param = {
	.port = 4,
	.number = 13,
	.platform_ops = GPIO_OPS,
	.extra = PWM_GPIO_EXTRA
};

struct no_os_gpio_init_param tmc6100_wl_gpio_param = {
	.port = 1,
	.number = 1,
	.platform_ops = GPIO_OPS,
	.extra = PWM_GPIO_EXTRA
};

struct no_os_pwm_init_param tmc6100_uh_pwm_param = {
	.id = TMC6100_PWMH_ID,
	.period_ns = PWM_SECTOR_PERIOD_NS,
	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_UH_EXTRA,
	.pwm_gpio = &tmc6100_uh_gpio_param,
};

struct no_os_pwm_init_param tmc6100_vh_pwm_param = {
	.id = TMC6100_PWMH_ID,
	.period_ns = PWM_SECTOR_PERIOD_NS,
	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_VH_EXTRA,
	.pwm_gpio = &tmc6100_vh_gpio_param,
};

struct no_os_pwm_init_param tmc6100_wh_pwm_param = {
	.id = TMC6100_PWMH_ID,
	.period_ns = PWM_SECTOR_PERIOD_NS,
	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_WH_EXTRA,
	.pwm_gpio = &tmc6100_wh_gpio_param,
};

struct no_os_pwm_init_param tmc6100_ul_pwm_param = {
	.id = TMC6100_PWML_ID,
	.period_ns = PWM_SECTOR_PERIOD_NS,
	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_UL_EXTRA,
	.pwm_gpio = &tmc6100_ul_gpio_param,
};

struct no_os_pwm_init_param tmc6100_vl_pwm_param = {
	.id = TMC6100_PWML_ID,
	.period_ns = PWM_SECTOR_PERIOD_NS,
	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_VL_EXTRA,
	.pwm_gpio = &tmc6100_vl_gpio_param,
};

struct no_os_pwm_init_param tmc6100_wl_pwm_param = {
	.id = TMC6100_PWML_ID,
	.period_ns = PWM_SECTOR_PERIOD_NS,
	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_WL_EXTRA,
	.pwm_gpio = &tmc6100_wl_gpio_param,
};

struct tmc6100_init_param tmc6100_ip = {
	.comm_param = &tmc6100_comm_param,
	.drv_en_param = &tmc6100_drv_en_param,
	.uh_pwm_param = &tmc6100_uh_pwm_param,
	.vh_pwm_param = &tmc6100_vh_pwm_param,
	.wh_pwm_param = &tmc6100_wh_pwm_param,
	.ul_pwm_param = &tmc6100_ul_pwm_param,
	.vl_pwm_param = &tmc6100_vl_pwm_param,
	.wl_pwm_param = &tmc6100_wl_pwm_param,
	.pwm_period_ns = PWM_SECTOR_PERIOD_NS,
};