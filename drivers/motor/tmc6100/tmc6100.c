#include "no_os_error.h"
#include "no_os_alloc.h"
#include "no_os_delay.h"
#include "stm32_pwm.h"
#include "tmc6100.h"

int tmc6100_reg_read(struct tmc6100_desc *desc, uint8_t reg, uint32_t *val)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->buff,
		.rx_buff = desc->buff,
		.bytes_number = 5,
		.cs_change = 1,
	};
	int ret;

	desc->buff[0] = reg;

	ret = no_os_spi_transfer(desc->comm_desc, &xfer, 1);
	if (ret)
		return ret;

	*val = no_os_get_unaligned_be32(&desc->buff[1]);

	return 0;
}

int tmc6100_reg_write(struct tmc6100_desc *desc, uint8_t reg, uint32_t val)
{
	struct no_os_spi_msg xfer = {
		.tx_buff = desc->buff,
		.rx_buff = desc->buff,
		.bytes_number = 5,
		.cs_change = 1,
	};

	desc->buff[0] = reg | TMC6100_RW_MASK;
	no_os_put_unaligned_be32(val, &desc->buff[1]);

	return no_os_spi_transfer(desc->comm_desc, &xfer, 1);
}

int tmc6100_reg_update(struct tmc6100_desc *desc, uint8_t reg, uint32_t mask,
		       uint32_t val)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, reg, &reg_val);
	if (ret)
		return ret;

	reg_val &= ~mask;
	reg_val |= no_os_field_prep(mask, val);

	return tmc6100_reg_write(desc, reg, reg_val);
}

int tmc6100_set_duty(struct tmc6100_desc *desc, uint16_t duty_u,
		     uint16_t duty_v, uint16_t duty_w)
{
	int ret;

	if (desc->ext_ctrl)
		return -EIO;

	if (duty_u > desc->pwm_period_ns || duty_v > desc->pwm_period_ns
	    || duty_w > desc->pwm_period_ns)
		return -EINVAL;

	no_os_pwm_set_duty_cycle(desc->uh_pwm_desc, duty_u);
	no_os_pwm_set_duty_cycle(desc->ul_pwm_desc, duty_u);
	no_os_pwm_set_duty_cycle(desc->vh_pwm_desc, duty_v);
	no_os_pwm_set_duty_cycle(desc->vl_pwm_desc, duty_v);
	no_os_pwm_set_duty_cycle(desc->wh_pwm_desc, duty_w);
	return no_os_pwm_set_duty_cycle(desc->wl_pwm_desc, duty_w);
}

int tmc6100_start_bldc_motor(struct tmc6100_desc *desc)
{
	if (desc->ext_ctrl)
		return -EIO;

	no_os_pwm_enable(desc->uh_pwm_desc);
	no_os_pwm_enable(desc->vh_pwm_desc);
	no_os_pwm_enable(desc->wh_pwm_desc);

	no_os_pwm_enable(desc->ul_pwm_desc);
	no_os_pwm_enable(desc->vl_pwm_desc);
	return no_os_pwm_enable(desc->wl_pwm_desc);
}

int tmc6100_stop_bldc_motor(struct tmc6100_desc *desc)
{
	if (desc->ext_ctrl)
		return -EIO;

	no_os_pwm_disable(desc->uh_pwm_desc);
	no_os_pwm_disable(desc->vh_pwm_desc);
	no_os_pwm_disable(desc->wh_pwm_desc);

	no_os_pwm_disable(desc->ul_pwm_desc);
	no_os_pwm_disable(desc->vl_pwm_desc);
	return no_os_pwm_disable(desc->wl_pwm_desc);
}

int tmc6100_get_fault(struct tmc6100_desc *desc, enum tmc6100_fault_sel *fault)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, TMC6100_GCONF_REG, &reg_val);
	if (ret)
		return ret;

	*fault = no_os_field_get(TMC6100_FAULT_MASK, reg_val);

	return 0;
}

int tmc6100_set_fault(struct tmc6100_desc *desc, enum tmc6100_fault_sel fault)
{
	return tmc6100_reg_update(desc, TMC6100_GCONF_REG, TMC6100_FAULT_MASK,
				  fault);
}

int tmc6100_sel_interface(struct tmc6100_desc *desc,
			  enum tmc6100_interface interface)
{
	int ret;

	ret = tmc6100_reg_update(desc, TMC6100_GCONF_REG, TMC6100_INT_MASK,
				 interface);
	if (ret)
		return ret;

	desc->interface = interface;

	return 0;
}

int tmc6100_read_status(struct tmc6100_desc *desc,
			enum tmc6100_status_sel status,
			bool *status_on)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, TMC6100_GSTAT_REG, &reg_val);
	if (ret)
		return ret;

	*status_on = no_os_field_get(TMC6100_STATUS_MASK(status), reg_val);

	return 0;
}

int tmc6100_set_short_sens(struct tmc6100_desc *desc,
			   enum tmc6100_short_sens_sel short_sens,
			   uint8_t sensivity)
{
	uint32_t mask;
	int ret;

	if (sensivity > TMC6100_SENS_MAX)
		return -EINVAL;

	switch (short_sens) {
	case TMC6100_SHORT2VSUPPLY_SENS:
		mask = TMC6100_S2VS_MASK;

		break;
	case TMC6100_SHORT2VGND_SENS:
		mask = TMC6100_S2GND_MASK;

		break;
	default:
		return -EINVAL;
	}

	return tmc6100_reg_update(desc, TMC6100_SHORT_CONF_REG, mask,
				  sensivity);
}

int tmc6100_get_short_sens(struct tmc6100_desc *desc,
			   enum tmc6100_short_sens_sel short_sens,
			   uint8_t *sensivity)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, TMC6100_SHORT_CONF_REG, &reg_val);
	if (ret)
		return ret;

	switch (short_sens) {
	case TMC6100_SHORT2VSUPPLY_SENS:
		*sensivity = no_os_field_get(TMC6100_S2VS_MASK, reg_val);

		return 0;
	case TMC6100_SHORT2VGND_SENS:
		*sensivity = no_os_field_get(TMC6100_S2GND_MASK, reg_val);

		return 0;
	default:
		return -EINVAL;
	}
}

int tmc6100_set_filter_short_bw(struct tmc6100_desc *desc,
				enum tmc6100_short_filter_bw filter_short_bw)
{
	return tmc6100_reg_update(desc, TMC6100_SHORT_CONF_REG,
				  TMC6100_SHORT_FILT_MASK, filter_short_bw);
}

int tmc6100_get_filter_short_bw(struct tmc6100_desc *desc,
				enum tmc6100_short_filter_bw *filter_short_bw)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, TMC6100_SHORT_CONF_REG, &reg_val);
	if (ret)
		return ret;

	*filter_short_bw = no_os_field_get(TMC6100_SHORT_FILT_MASK, reg_val);

	return 0;
}

int tmc6100_enable_short(struct tmc6100_desc *desc,
			 enum tmc6100_short_sens_sel short_sens,
			 bool enable)
{
	uint32_t mask;

	switch (short_sens) {
	case TMC6100_SHORT2VSUPPLY_SENS:
		mask = TMC6100_S2VS_EN_MASK;

		break;
	case TMC6100_SHORT2VGND_SENS:
		mask = TMC6100_S2GND_EN_MASK;

		break;
	default:
		return -EINVAL;
	}

	return tmc6100_reg_update(desc, TMC6100_SHORT_CONF_REG, mask, !enable);
}

int tmc6100_set_bbm_tme(struct tmc6100_desc *desc, uint8_t bbm_time)
{
	if (desc->interface == TMC6100_L_H_INDIVIDUAL)
		return -EINVAL;

	if (bbm_time > TMC6100_BBM_MAX)
		return -EINVAL;

	return tmc6100_reg_update(desc, TMC6100_DRV_CONF_REG, TMC6100_BBM_MASK,
				  bbm_time);
}

int tmc6100_get_bbm_tme(struct tmc6100_desc *desc, uint8_t *bbm_time)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, TMC6100_DRV_CONF_REG, &reg_val);
	if (ret)
		return ret;

	*bbm_time = no_os_field_get(TMC6100_BBM_MASK, reg_val);

	return 0;
}

int tmc6100_set_drv_strength(struct tmc6100_desc *desc,
			     enum tmc6100_drv_strength drv_strength)
{
	return tmc6100_reg_update(desc, TMC6100_DRV_CONF_REG,
				  TMC6100_DRV_STRENGTH_MASK, drv_strength);
}

int tmc6100_get_drv_strength(struct tmc6100_desc *desc,
			     enum tmc6100_drv_strength *drv_strength)
{
	uint32_t reg_val;
	int ret;

	ret = tmc6100_reg_read(desc, TMC6100_DRV_CONF_REG, &reg_val);
	if (ret)
		return ret;

	*drv_strength = no_os_field_get(TMC6100_DRV_STRENGTH_MASK, reg_val);

	return 0;
}

int tmc6100_init(struct tmc6100_desc **desc,
		 struct tmc6100_init_param *init_param)
{
	struct tmc6100_desc *descriptor;
	uint32_t reg_val;
	int ret, i;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = no_os_spi_init(&descriptor->comm_desc, init_param->comm_param);
	if (ret)
		goto free_tmc6100;

	descriptor->ext_ctrl = init_param->ext_ctrl;
	ret = tmc6100_sel_interface(descriptor, TMC6100_L_H_INDIVIDUAL);
	if (ret)
		goto wl_pwm_error;

	descriptor->pwm_period_ns = init_param->pwm_period_ns;
	descriptor->interface = TMC6100_L_H_INDIVIDUAL;
	ret = no_os_gpio_get(&descriptor->drv_en_desc, init_param->drv_en_param);
	if (ret)
		goto wl_pwm_error;

	ret = no_os_gpio_direction_output(descriptor->drv_en_desc, NO_OS_GPIO_HIGH);
	if (ret)
		goto drv_en_error;

	if (!init_param->ext_ctrl) {
		init_param->uh_pwm_param->period_ns = init_param->pwm_period_ns;
		init_param->uh_pwm_param->duty_cycle_ns = init_param->pwm_period_ns / 2;
		ret = no_os_pwm_init(&descriptor->uh_pwm_desc, init_param->uh_pwm_param);
		if (ret)
			goto spi_error;

		ret = no_os_pwm_disable(descriptor->uh_pwm_desc);
		if (ret)
			goto uh_pwm_error;

		init_param->ul_pwm_param->period_ns = init_param->pwm_period_ns;
		init_param->ul_pwm_param->duty_cycle_ns = init_param->pwm_period_ns / 2;
		ret = no_os_pwm_init(&descriptor->ul_pwm_desc, init_param->ul_pwm_param);
		if (ret)
			goto uh_pwm_error;

		ret = no_os_pwm_disable(descriptor->ul_pwm_desc);
		if (ret)
			goto ul_pwm_error;

		init_param->vh_pwm_param->period_ns = init_param->pwm_period_ns;
		init_param->vh_pwm_param->duty_cycle_ns = init_param->pwm_period_ns / 2;
		ret = no_os_pwm_init(&descriptor->vh_pwm_desc, init_param->vh_pwm_param);
		if (ret)
			goto ul_pwm_error;

		ret = no_os_pwm_disable(descriptor->vh_pwm_desc);
		if (ret)
			goto vh_pwm_error;

		init_param->vl_pwm_param->period_ns = init_param->pwm_period_ns;
		init_param->vl_pwm_param->duty_cycle_ns = init_param->pwm_period_ns / 2;
		ret = no_os_pwm_init(&descriptor->vl_pwm_desc, init_param->vl_pwm_param);
		if (ret)
			goto vh_pwm_error;

		ret = no_os_pwm_disable(descriptor->vl_pwm_desc);
		if (ret)
			goto vl_pwm_error;

		init_param->wh_pwm_param->period_ns = init_param->pwm_period_ns;
		init_param->wh_pwm_param->duty_cycle_ns = init_param->pwm_period_ns / 2;
		ret = no_os_pwm_init(&descriptor->wh_pwm_desc, init_param->wh_pwm_param);
		if (ret)
			goto vl_pwm_error;

		ret = no_os_pwm_disable(descriptor->wh_pwm_desc);
		if (ret)
			goto wh_pwm_error;

		init_param->wl_pwm_param->period_ns = init_param->pwm_period_ns;
		init_param->wl_pwm_param->duty_cycle_ns = init_param->pwm_period_ns / 2;
		ret = no_os_pwm_init(&descriptor->wl_pwm_desc, init_param->wl_pwm_param);
		if (ret)
			goto wh_pwm_error;

		ret = no_os_pwm_disable(descriptor->wl_pwm_desc);
		if (ret)
			goto wl_pwm_error;
	}

	ret = tmc6100_reg_read(descriptor, TMC6100_GSTAT_REG, &reg_val);
	if (ret)
		goto wl_pwm_error;

	ret = tmc6100_reg_write(descriptor, TMC6100_GSTAT_REG, reg_val);
	if (ret)
		goto wl_pwm_error;

	*desc = descriptor;

	return 0;


wl_pwm_error:
	if (init_param->ext_ctrl)
		goto drv_en_error;

	no_os_pwm_remove(descriptor->wl_pwm_desc);
wh_pwm_error:
	no_os_pwm_remove(descriptor->wh_pwm_desc);
vl_pwm_error:
	no_os_pwm_remove(descriptor->vl_pwm_desc);
vh_pwm_error:
	no_os_pwm_remove(descriptor->vh_pwm_desc);
ul_pwm_error:
	no_os_pwm_remove(descriptor->ul_pwm_desc);
uh_pwm_error:
	no_os_pwm_remove(descriptor->uh_pwm_desc);
drv_en_error:
	no_os_gpio_remove(descriptor->drv_en_desc);
spi_error:
	no_os_spi_remove(descriptor->comm_desc);
free_tmc6100:
	no_os_free(descriptor);

	return ret;
}

int tmc6100_remove(struct tmc6100_desc *desc)
{
	int i;

	no_os_gpio_remove(desc->drv_en_desc);

	tmc6100_stop_bldc_motor(desc);
	no_os_pwm_remove(desc->wl_pwm_desc);
	no_os_pwm_remove(desc->wh_pwm_desc);
	no_os_pwm_remove(desc->vl_pwm_desc);
	no_os_pwm_remove(desc->vh_pwm_desc);
	no_os_pwm_remove(desc->ul_pwm_desc);
	no_os_pwm_remove(desc->uh_pwm_desc);

	no_os_spi_remove(desc->comm_desc);
	no_os_free(desc);

	return 0;
}
