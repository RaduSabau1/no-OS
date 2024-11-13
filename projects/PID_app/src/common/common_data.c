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
	.extra = SPI_EXTRA,
	.max_speed_hz = TMC4671_SPI_BAUDRATE,
	.platform_ops = SPI_OPS,
	.chip_select = TMC4671_SPI_CS,
	.mode = NO_OS_SPI_MODE_3,
};

struct no_os_gpio_init_param tmc4671_rstn_param = {
	.port = TMC4671_GPIO_RSTN_PORT_NUM,
	.pull = NO_OS_PULL_NONE,
	.number = TMC4671_GPIO_RSTN_PIN_NUM,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA
};

struct tmc4671_init_param tmc4671_ip = {
	.spi_param = &tmc4671_spi_param,
	.rstn_param = &tmc4671_rstn_param,
	.motor_type = TMC4671_THREE_PHASE_BLDC,
	.no_pole_pairs = 4,
};

struct no_os_spi_init_param tmc6100_comm_param = {
	.device_id = TMC6100_SPI_DEVICE_ID,
	.extra = SPI_EXTRA,
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

// struct no_os_gpio_init_param tmc6100_uh_param = {
// 	.port = TMC6100_GPIO_UH_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = TMC6100_GPIO_UH_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };

// struct no_os_gpio_init_param tmc6100_ul_param = {
// 	.port = TMC6100_GPIO_UL_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = TMC6100_GPIO_UL_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };

// struct no_os_gpio_init_param tmc6100_vh_param = {
// 	.port = TMC6100_GPIO_VH_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = TMC6100_GPIO_VH_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };

// struct no_os_gpio_init_param tmc6100_vl_param = {
// 	.port = TMC6100_GPIO_VL_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = TMC6100_GPIO_VL_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };

// struct no_os_gpio_init_param tmc6100_wh_param = {
// 	.port = TMC6100_GPIO_WH_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = TMC6100_GPIO_WH_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };

// struct no_os_gpio_init_param tmc6100_wl_param = {
// 	.port = TMC6100_GPIO_WL_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = TMC6100_GPIO_WL_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };

struct no_os_gpio_init_param tmc6100_uvw_enable_param = {
	.port = TMC6100_UVW_ENABLE_PORT_NUM,
	.pull = NO_OS_PULL_NONE,
	.number = TMC6100_UVW_ENABLE_PIN_NUM,
	.platform_ops = GPIO_OPS,
	.extra = GPIO_EXTRA
};

struct no_os_pwm_init_param tmc6100_u_pwm_param = {
	.id = TMC6100_U_PWM_ID,
	.period_ns = 40000,
	.duty_cycle_ns = 10000,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_EXTRA,
};

struct no_os_pwm_init_param tmc6100_v_pwm_param = {
	.id = TMC6100_V_PWM_ID,
	.period_ns = 40000,
	.duty_cycle_ns = 20000,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_EXTRA,
};

struct no_os_pwm_init_param tmc6100_w_pwm_param = {
	.id = TMC6100_W_PWM_ID,
	.period_ns = 40000,
	.duty_cycle_ns = 20000,
	.polarity = NO_OS_PWM_POLARITY_HIGH,
	.platform_ops = PWM_OPS,
	.extra = PWM_EXTRA,
};


struct tmc6100_init_param tmc6100_ip = {
	.comm_param = &tmc6100_comm_param,
	.drv_en_param = &tmc6100_drv_en_param,
	// .spe_param = &tmc6100_spe_param,
	// .uh_param = &tmc6100_uh_param,
	// .ul_param = &tmc6100_ul_param,
	// .vh_param = &tmc6100_vh_param,
	// .vl_param = &tmc6100_vl_param,
	// .wh_param = &tmc6100_wh_param,
	// .wl_param = &tmc6100_wl_param,
	.uvw_enable_param = &tmc6100_uvw_enable_param,
	.u_pwm_param = &tmc6100_u_pwm_param,
	.v_pwm_param = &tmc6100_v_pwm_param,
	.w_pwm_param = &tmc6100_w_pwm_param,
	.interface = TMC6100_L_H_INDIVIDUAL,
};

// struct no_os_pwm_init_param pwm_sector_ip = {
// 	.id = PWM_SECTOR_ID,
// 	.period_ns = PWM_SECTOR_PERIOD_NS,
// 	.duty_cycle_ns = PWM_SECTOR_PERIOD_NS / 2,
// 	.polarity = NO_OS_PWM_POLARITY_HIGH,
// 	.platform_ops = PWM_OPS,
// 	.extra = PWM_EXTRA,
// };

// struct no_os_gpio_init_param pwmin_gpio_ip = {
// 	.port = PID_GPIO_PWMIN_PORT_NUM,
// 	.pull = NO_OS_PULL_NONE,
// 	.number = PID_GPIO_PWMIN_PIN_NUM,
// 	.platform_ops = GPIO_OPS,
// 	.extra = GPIO_EXTRA
// };
