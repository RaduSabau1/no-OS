/***************************************************************************//**
 *   @file   tmc6100.h
 *   @brief  Header file of TMC6100 gate driver.
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef _TMC6100_H_
#define _TMC6100_H_
#include <stdint.h>

#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

#define TMC6100_GCONF_REG		0x00
#define TMC6100_GSTAT_REG		0x01
#define TMC6100_IOIN_REG		0x04
#define TMC6100_OTP_PROG_REG		0x06
#define TMC6100_OTP_READ_REG		0x07
#define TMC6100_FACTORY_CONF_REG	0x08
#define TMC6100_SHORT_CONF_REG		0x09
#define TMC6100_DRV_CONF_REG		0x0A

#define TMC6100_RW_MASK			NO_OS_BIT(7)
#define TMC6100_FAULT_MASK		NO_OS_BIT(2)
#define TMC6100_INT_MASK		NO_OS_BIT(1)
#define TMC6100_STATUS_MASK(x)		NO_OS_BIT(x)
#define TMC6100_S2VS_MASK		NO_OS_GENMASK(3, 0)
#define TMC6100_S2GND_MASK		NO_OS_GENMASK(11, 8)
#define TMC6100_SHORT_FILT_MASK		NO_OS_GENMASK(17, 16)
#define TMC6100_S2GND_EN_MASK		NO_OS_BIT(29)
#define TMC6100_S2VS_EN_MASK		NO_OS_BIT(30)
#define TMC6100_BBM_MASK		NO_OS_GENMASK(4, 0)
#define TMC6100_DRV_STRENGTH_MASK	NO_OS_GENMASK(19, 18)

#define TMC6100_SENS_MAX		0x0F
#define TMC6100_BBM_MAX			0x1F

enum tmc6100_fault_sel {
	TMC6100_FAULT_OVC_OTP_ONLY,
	TMC6100_FAULT_ALL,
};

enum tmc6100_interface {
	TMC6100_L_H_INDIVIDUAL,
	TMC6100_H_CONTROL_L_ENABLE,
};

enum tmc6100_status_sel {
	TMC6100_RESET_STATUS,
	TMC6100_DRVOTPW_STATUS,
	TMC6100_DRVOT_STATUS,
	TMC6100_UVCP_STATUS,
	TMC6100_SHORTDET_U_STATUS,
	TMC6100_SHORT2GNDU_STATUS,
	TMC6100_SHORT2VSU_STATUS,
	TMC6100_SHORTDET_V_STATUS = 8,
	TMC6100_SHORT2GNDV_STATUS,
	TMC6100_SHORT2VSV_STATUS,
	TMC6100_SHORTDET_W_STATUS = 12,
	TMC6100_SHORT2GNDW_STATUS,
	TMC6100_SHORT2VSW_STATUS,
};

enum tmc6100_short_sens_sel {
	TMC6100_SHORT2VSUPPLY_SENS,
	TMC6100_SHORT2VGND_SENS,
};

enum tmc6100_short_filter_bw {
	TMC6100_SHORT_FILTER_BW_100NS,
	TMC6100_SHORT_FILTER_BW_1US,
	TMC6100_SHORT_FILTER_BW_2US,
	TMC6100_SHORT_FILTER_BW_3US,
};

enum tmc6100_drv_strength {
	TMC6100_DRV_STRENGTH_WEAK,
	TMC6100_DRV_STRENGTH_WEAK_TC,
	TMC6100_DRV_STRENGTH_MEDIUM,
	TMC6100_DRV_STRENGTH_STRONG,
};

struct tmc6100_init_param {
	struct no_os_spi_init_param *comm_param;
	struct no_os_gpio_init_param *drv_en_param;
};

struct tmc6100_desc {
	struct no_os_spi_desc *comm_desc;
	struct no_os_gpio_desc *drv_en_desc;
	uint8_t buff[6];
	enum tmc6100_interface interface;
};

int tmc6100_reg_read(struct tmc6100_desc *desc, uint8_t reg, uint32_t *val);

int tmc6100_reg_write(struct tmc6100_desc *desc, uint8_t reg, uint32_t val);

int tmc6100_reg_update(struct tmc6100_desc *desc, uint8_t reg, uint32_t mask,
		       uint32_t val);

int tmc6100_get_fault(struct tmc6100_desc *desc, enum tmc6100_fault_sel *fault);

int tmc6100_set_fault(struct tmc6100_desc *desc, enum tmc6100_fault_sel fault);

int tmc6100_sel_interface(struct tmc6100_desc *desc,
			  enum tmc6100_interface interface);

int tmc6100_read_status(struct tmc6100_desc *desc,
			enum tmc6100_status_sel status,
			bool *status_on);

int tmc6100_set_short_sens(struct tmc6100_desc *desc,
			   enum tmc6100_short_sens_sel short_sens,
			   uint8_t sensivity);

int tmc6100_get_short_sens(struct tmc6100_desc *desc,
			   enum tmc6100_short_sens_sel short_sens,
			   uint8_t *sensivity);

int tmc6100_set_filter_short_bw(struct tmc6100_desc *desc,
				enum tmc6100_short_filter_bw filter_short_bw);

int tmc6100_get_filter_short_bw(struct tmc6100_desc *desc,
				enum tmc6100_short_filter_bw *filter_short_bw);

int tmc6100_enable_short(struct tmc6100_desc *desc,
			 enum tmc6100_short_sens_sel short_sens,
			 bool enable);

int tmc6100_set_bbm_tme(struct tmc6100_desc *desc, uint8_t bbm_time);

int tmc6100_get_bbm_tme(struct tmc6100_desc *desc, uint8_t *bbm_time);

int tmc6100_set_drv_strength(struct tmc6100_desc *desc,
			     enum tmc6100_drv_strength drv_strength);

int tmc6100_get_drv_strength(struct tmc6100_desc *desc,
			     enum tmc6100_drv_strength *drv_strength);

int tmc6100_init(struct tmc6100_desc **desc,
		 struct tmc6100_init_param *init_param);

int tmc6100_remove(struct tmc6100_desc *desc);

#endif
