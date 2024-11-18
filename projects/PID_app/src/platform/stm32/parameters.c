#include "parameters.h"

struct stm32_uart_init_param pid_uart_extra = {
	.huart = UART_INSTANCE,
};

struct stm32_gpio_init_param pid_gpio_extra = {
	.mode = GPIO_MODE_OUTPUT_PP,
	.speed = GPIO_SPEED_HIGH,
};

struct stm32_gpio_init_param pid_pwm_gpio_extra = {
	.mode = GPIO_MODE_AF_PP,
	.speed = GPIO_SPEED_HIGH,
	.alternate = GPIO_AF1_TIM1,
};

struct stm32_spi_init_param pid_spi_extra_tmc4671 = {
	.chip_select_port = SPI_CS_PORT_TMC4671,
	.get_input_clock = HAL_RCC_GetHCLKFreq,
};

struct stm32_spi_init_param pid_spi_extra_tmc6100 = {
	.chip_select_port = SPI_CS_PORT_TMC6100,
	.get_input_clock = HAL_RCC_GetHCLKFreq,
};

struct stm32_pwm_init_param pid_pwm_uh_extra = {
	.prescaler = 1,
	.timer_autoreload = true,
	.mode = TIM_OCMODE_PWM1,
	.timer_chn = 1,
	.complementary_channel = false,
	.get_timer_clock = HAL_RCC_GetPCLK2Freq,
	.clock_divider = 1,
	.trigger_output = PWM_TRGO_UPDATE,
	.timer_mode = TIM_COUNTERMODE_CENTERALIGNED1,
};

struct stm32_pwm_init_param pid_pwm_vh_extra = {
	.prescaler = 1,
	.timer_autoreload = true,
	.mode = TIM_OCMODE_PWM1,
	.timer_chn = 2,
	.complementary_channel = false,
	.get_timer_clock = HAL_RCC_GetPCLK2Freq,
	.clock_divider = 1,
	.trigger_output = PWM_TRGO_UPDATE,
	.timer_mode = TIM_COUNTERMODE_CENTERALIGNED1,
};

struct stm32_pwm_init_param pid_pwm_wh_extra = {
	.prescaler = 1,
	.timer_autoreload = true,
	.mode = TIM_OCMODE_PWM1,
	.timer_chn = 3,
	.complementary_channel = false,
	.get_timer_clock = HAL_RCC_GetPCLK2Freq,
	.clock_divider = 1,
	.trigger_output = PWM_TRGO_UPDATE,
	.timer_mode = TIM_COUNTERMODE_CENTERALIGNED1,
};

struct stm32_pwm_init_param pid_pwm_ul_extra = {
	.prescaler = 1,
	.timer_autoreload = true,
	.mode = TIM_OCMODE_PWM1,
	.timer_chn = 1,
	.complementary_channel = true,
	.get_timer_clock = HAL_RCC_GetPCLK2Freq,
	.clock_divider = 1,
	.trigger_output = PWM_TRGO_UPDATE,
	.timer_mode = TIM_COUNTERMODE_CENTERALIGNED1,
};

struct stm32_pwm_init_param pid_pwm_vl_extra = {
	.prescaler = 1,
	.timer_autoreload = true,
	.mode = TIM_OCMODE_PWM1,
	.timer_chn = 2,
	.complementary_channel = true,
	.get_timer_clock = HAL_RCC_GetPCLK2Freq,
	.clock_divider = 1,
	.trigger_output = PWM_TRGO_UPDATE,
	.timer_mode = TIM_COUNTERMODE_CENTERALIGNED1,
};

struct stm32_pwm_init_param pid_pwm_wl_extra = {
	.prescaler = 1,
	.timer_autoreload = true,
	.mode = TIM_OCMODE_PWM1,
	.timer_chn = 3,
	.complementary_channel = true,
	.get_timer_clock = HAL_RCC_GetPCLK2Freq,
	.clock_divider = 1,
	.trigger_output = PWM_TRGO_UPDATE,
	.timer_mode = TIM_COUNTERMODE_CENTERALIGNED1,
};
