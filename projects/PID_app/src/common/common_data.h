#ifndef __COMMON_DATA_H__
#define __COMMON_DATA_H__

#include "platform_includes.h"
#include "no_os_pwm.h"
#include "tmc6100.h"
#include "tmc4671.h"

extern struct no_os_uart_init_param pid_app_uart_ip;

extern struct no_os_spi_init_param tmc4671_spi_param;
extern struct no_os_gpio_init_param tmc4671_rstn_param;
extern struct tmc4671_init_param tmc4671_ip;

extern struct no_os_spi_init_param tmc6100_comm_param;
extern struct no_os_gpio_init_param tmc6100_drv_en_param;
extern struct tmc6100_init_param tmc6100_ip;

#endif /* __COMMON_DATA_H__ */
