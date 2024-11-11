#include "tmc4671.h"
#include "no_os_error.h"

static uint32_t tmc4671_raw_angle_to_degree(uint32_t raw_angle)
{
	return (raw_angle * 360) / 0xFFFF;
}

int tmc4671_reg_read(struct tmc4671_desc *desc, uint8_t reg, uint32_t *val)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->buff,
		.rx_buff = desc->buff,
		.bytes_number = 5,
		.cs_change = 1,
	};
	int ret;

	desc->buff[0] = reg;

	ret = no_os_spi_transfer(desc->spi_desc, &xfer, 1);
	if (ret)
		return ret;

	*val = no_os_get_unaligned_be32(&desc->buff[1]);

	return 0;
}

int tmc4671_reg_write(struct tmc4671_desc *desc, uint8_t reg, uint32_t val)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->buff,
		.rx_buff = desc->buff,
		.bytes_number = 5,
		.cs_change = 1,
	};

	desc->buff[0] = reg | TMC4671_RW_MASK;
	no_os_put_unaligned_be32(val, &desc->buff[1]);

	return no_os_spi_transfer(desc->spi_desc, &xfer, 1);
}

int tmc4671_reg_update(struct tmc4671_desc *desc, uint8_t reg, uint32_t mask,
		       uint32_t val)
{
	uint32_t reg_val;
	int ret;

	ret = tmc4671_reg_read(desc, reg, &reg_val);
	if (ret)
		return ret;

	reg_val &= ~mask;
	reg_val |= no_os_field_prep(mask, val);

	return tmc4671_reg_write(desc, reg, reg_val);
}

int tmc4671_velocity_sel(struct tmc4671_desc *desc, enum tmc4671_source_sel sel)
{
	return tmc4671_reg_update(desc, TMC4671_VELOCITY_SELECTION_REG,
				  TMC4671_VELOCITY_SELECTION_MASK, sel);
}

int tmc4671_position_sel(struct tmc4671_desc *desc,
			 enum tmc4671_source_sel sel)
{
	return tmc4671_reg_update(desc, TMC4671_POSITION_SELECTION_REG,
				  TMC4671_POSITION_SELECTION_MASK, sel);
}

int tmc4671_phi_e_sel(struct tmc4671_desc *desc, enum tmc4671_source_sel sel)
{
	if (sel == TMC4671_PHI_E_SELECTION)
		return -EINVAL;

	return tmc4671_reg_update(desc, TMC4671_PHI_E_SELECTION_REG,
				  TMC4671_PHI_E_SELECTION_MASK, sel);
}

int tmc4671_read_angle(struct tmc4671_desc *desc, uint32_t *angle)
{
	uint32_t reg_val;
	int ret;

	ret = tmc4671_reg_read(desc, TMC4671_PHI_E_REG, &reg_val);
	if (ret)
		return ret;

	reg_val = no_os_field_get(TMC4671_PHI_E_MASK, reg_val);

	*angle = reg_val;

	return 0;
}

int tmc4671_read_velocity(struct tmc4671_desc *desc, int32_t *velocity)
{
	uint32_t reg_val;
	int ret;

	ret = tmc4671_reg_read(desc, TMC4671_PID_VELOCITY_ACTUAL_REG, &reg_val);
	if (ret)
		return ret;

	*velocity = ((int32_t)reg_val / (desc->no_pole_pairs));

	return 0;
}

int tmc4671_set_hall_mode(struct tmc4671_desc *desc,
			  enum tmc4671_hall_mode_sel sel,
			  bool enable)
{
	return tmc4671_reg_update(desc, TMC4671_HALL_MODE_REG,
				  TMC4671_HALL_MODE_MASK(sel), enable);
}

int tmc4671_get_hall_mode(struct tmc4671_desc *desc,
			  enum tmc4671_hall_mode_sel sel,
			  bool *enable)
{
	uint32_t reg_val;
	int ret;

	ret = tmc4671_reg_read(desc, TMC4671_HALL_MODE_REG, &reg_val);
	if (ret)
		return ret;

	*enable = no_os_field_get(TMC4671_HALL_MODE_MASK(sel), reg_val);

	return 0;
}

int tmc4671_set_pwm_freq(struct tmc4671_desc *desc, uint32_t freq)
{
	if (freq < 24414)
		return -EINVAL;

	return tmc4671_reg_update(desc, TMC4671_PWM_MAXCNT_REG, TMC4671_PWM_MAXCNT_MASK,
				  NO_OS_DIV_ROUND_CLOSEST_ULL(100000000UL, freq) - 1);
}

int tmc4671_set_pwm_bbm(struct tmc4671_desc *desc, uint8_t bbm)
{
	return tmc4671_reg_write(desc, TMC4671_PWM_BBM_H_BBM_L_REG,
				 (uint32_t)((uint16_t)(bbm << 8) | bbm));
}

int tmc4671_set_pwm_sv_chop(struct tmc4671_desc *desc,
			    enum tmc4671_pwm_sv_chop_sel sv_chop)
{
	return tmc4671_reg_update(desc, TMC4671_PWM_SV_CHOP_REG,
				  TMC4671_PWM_SV_CHOP_MASK, sv_chop);
}

int tmc4671_set_mode_motion(struct tmc4671_desc *desc,
			    enum tmc4671_hall_mode_motion_sel mode_motion)
{
	return tmc4671_reg_update(desc, TMC4671_MODE_RAMP_MODE_MOTION_REG,
				  TMC4671_MODE_MOTION_MASK, mode_motion);
}

int tmc4671_init(struct tmc4671_desc **desc,
		 struct tmc4671_init_param *init_param)
{
	struct tmc4671_desc *descriptor;
	uint32_t reg_val;
	int ret;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = no_os_spi_init(&descriptor->spi_desc, init_param->spi_param);
	if (ret)
		goto err_tmc4671;

	ret = no_os_gpio_get_optional(&descriptor->rstn_desc, init_param->rstn_param);
	if (ret)
		goto err_spi;

	if (descriptor->rstn_desc) {
		ret = no_os_gpio_direction_output(descriptor->rstn_desc,
						  NO_OS_GPIO_HIGH);
		if (ret)
			goto err_rstn;
	}

	reg_val = no_os_field_prep(TMC4671_MOTOR_TYPE_MASK,
				   init_param->motor_type) |
		  init_param->no_pole_pairs;

	ret = tmc4671_reg_write(descriptor, TMC4671_MOTOR_TYPE_N_POLE_PAIRS_REG,
				reg_val);
	if (ret)
		goto err_rstn;

	descriptor->motor_type = init_param->motor_type;
	descriptor->no_pole_pairs = init_param->no_pole_pairs;

	*desc = descriptor;

	return 0;

err_rstn:
	no_os_gpio_remove(descriptor->rstn_desc);
err_spi:
	no_os_spi_remove(descriptor->spi_desc);
err_tmc4671:
	no_os_free(descriptor);

	return ret;
}

int tmc4671_remove(struct tmc4671_desc *desc)
{
	no_os_gpio_remove(desc->rstn_desc);
	no_os_spi_remove(desc->spi_desc);
	no_os_free(desc);

	return 0;
}
