#include "no_os_error.h"
#include "tmc6100.h"

int tmc6100_reg_read(struct tmc6100_desc *desc, uint8_t reg, uint32_t *val)
{
	struct no_os_spi_msg xfer = {
		.bytes_number = 5,
		.cs_change = 1,
	};
	uint8_t rx_data[5];
	int ret;

	xfer.tx_buff = &reg;
	xfer.rx_buff = rx_data;

	ret = no_os_spi_transfer(desc->comm_desc, &xfer, 1);
	if (ret)
		return ret;

	*val = no_os_get_unaligned_be32(&rx_data[1]);

	return 0;
}

int tmc6100_reg_write(struct tmc6100_desc *desc, uint8_t reg, uint32_t val)
{
	struct no_os_spi_msg xfer = {
		.bytes_number = 5,
		.cs_change = 1,
	};
	uint8_t tx_data[5];

	tx_data[0] = reg | TMC6100_RW_MASK;
	no_os_put_unaligned_be32(val, &tx_data[1]);

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

int tmc6100_bldc_create_seq(struct tmc6100_desc *desc,
			    enum tmc6100_out_pin_sel phase_start,
			    uint8_t *sequence)
{
	switch (phase_start) {
	case TMC6100_UH:
		sequence[0] = TMC6100_BLDC_UH_WL_SEQ;
		sequence[1] = TMC6100_BLDC_VH_WL_SEQ;
		sequence[2] = TMC6100_BLDC_UL_VH_SEQ;
		sequence[3] = TMC6100_BLDC_UL_WH_SEQ;
		sequence[4] = TMC6100_BLDC_VL_WH_SEQ;
		sequence[5] = TMC6100_BLDC_UH_VL_SEQ;

		return 0;
	case TMC6100_UL:
		sequence[3] = TMC6100_BLDC_UH_WL_SEQ;
		sequence[4] = TMC6100_BLDC_VH_WL_SEQ;
		sequence[5] = TMC6100_BLDC_UL_VH_SEQ;
		sequence[0] = TMC6100_BLDC_UL_WH_SEQ;
		sequence[1] = TMC6100_BLDC_VL_WH_SEQ;
		sequence[2] = TMC6100_BLDC_UH_VL_SEQ;

		return 0;
	case TMC6100_VH:
		sequence[4] = TMC6100_BLDC_UH_WL_SEQ;
		sequence[5] = TMC6100_BLDC_VH_WL_SEQ;
		sequence[0] = TMC6100_BLDC_UL_VH_SEQ;
		sequence[1] = TMC6100_BLDC_UL_WH_SEQ;
		sequence[2] = TMC6100_BLDC_VL_WH_SEQ;
		sequence[3] = TMC6100_BLDC_UH_VL_SEQ;

		return 0;
	case TMC6100_VL:
		sequence[1] = TMC6100_BLDC_UH_WL_SEQ;
		sequence[2] = TMC6100_BLDC_VH_WL_SEQ;
		sequence[3] = TMC6100_BLDC_UL_VH_SEQ;
		sequence[4] = TMC6100_BLDC_UL_WH_SEQ;
		sequence[5] = TMC6100_BLDC_VL_WH_SEQ;
		sequence[0] = TMC6100_BLDC_UH_VL_SEQ;

		return 0;
	case TMC6100_WH:
		sequence[2] = TMC6100_BLDC_UH_WL_SEQ;
		sequence[3] = TMC6100_BLDC_VH_WL_SEQ;
		sequence[4] = TMC6100_BLDC_UL_VH_SEQ;
		sequence[5] = TMC6100_BLDC_UL_WH_SEQ;
		sequence[0] = TMC6100_BLDC_VL_WH_SEQ;
		sequence[1] = TMC6100_BLDC_UH_VL_SEQ;

		return 0;
	case TMC6100_WL:
		sequence[5] = TMC6100_BLDC_UH_WL_SEQ;
		sequence[0] = TMC6100_BLDC_VH_WL_SEQ;
		sequence[1] = TMC6100_BLDC_UL_VH_SEQ;
		sequence[2] = TMC6100_BLDC_UL_WH_SEQ;
		sequence[3] = TMC6100_BLDC_VL_WH_SEQ;
		sequence[4] = TMC6100_BLDC_UH_VL_SEQ;;

		return 0;
	default:
		return -EINVAL;
	}
}

int tmc6100_bldc_sel_sector(struct tmc6100_desc *desc,
			    enum tmc6100_bldc_sector sector,
			    uint8_t *sequence)
{
	uint32_t prev_sec;
	int ret, i;

	if (sector == TMC6100_BLDC_SECTOR_1)
		prev_sec = TMC6100_BLDC_SECTOR_6;
	else
		prev_sec = sector - 1;

	ret = no_os_gpio_set_value(desc->wvu_desc[no_os_find_first_set_bit(
					   sequence[prev_sec])], NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	ret = no_os_gpio_set_value(desc->wvu_desc[no_os_find_last_set_bit(
					   sequence[prev_sec])], NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	ret = no_os_gpio_set_value(desc->wvu_desc[no_os_find_first_set_bit(
					   sequence[sector])], NO_OS_GPIO_HIGH);
	if (ret)
		return ret;

	return no_os_gpio_set_value(desc->wvu_desc[no_os_find_last_set_bit(
					    sequence[sector])], NO_OS_GPIO_HIGH);
}

int tmc6100_read_fault(struct tmc6100_desc *desc,
		       enum tmc6100_fault_sel fault,
		       bool *fault_on);

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

int tmc6100_set_drv_strength(struct tmc6100_desc *desc, uint8_t drv_strength);

int tmc6100_get_drv_strengthe(struct tmc6100_desc *desc, uint8_t *drv_strength);

int tmc6100_init(struct tmc6100_desc **desc,
		 struct tmc6100_init_param *init_param)
{
	struct tmc6100_desc *descriptor;
	int ret;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	ret = no_os_spi_init(&descriptor->comm_desc, init_param->comm_param);
	if (ret)
		goto free_tmc6100;

	ret = no_os_gpio_get(&descriptor->wvu_desc[0], init_param->wl_param);
	if (ret)
		goto spi_error;

	ret = no_os_gpio_direction_output(descriptor->wvu_desc[0], NO_OS_GPIO_LOW);
	if (ret)
		goto wl_error;

	ret = no_os_gpio_get(&descriptor->wvu_desc[1], init_param->wh_param);
	if (ret)
		goto wl_error;

	ret = no_os_gpio_direction_output(descriptor->wvu_desc[1], NO_OS_GPIO_LOW);
	if (ret)
		goto wh_error;

	ret = no_os_gpio_get(&descriptor->wvu_desc[2], init_param->vl_param);
	if (ret)
		goto wh_error;

	ret = no_os_gpio_direction_output(descriptor->wvu_desc[2], NO_OS_GPIO_LOW);
	if (ret)
		goto vl_error;

	ret = no_os_gpio_get(&descriptor->wvu_desc[3], init_param->vh_param);
	if (ret)
		goto vl_error;

	ret = no_os_gpio_direction_output(descriptor->wvu_desc[3], NO_OS_GPIO_LOW);
	if (ret)
		goto vh_error;

	ret = no_os_gpio_get(&descriptor->wvu_desc[4], init_param->ul_param);
	if (ret)
		goto vh_error;

	ret = no_os_gpio_direction_output(descriptor->wvu_desc[4], NO_OS_GPIO_LOW);
	if (ret)
		goto ul_error;

	ret = no_os_gpio_get(&descriptor->wvu_desc[5], init_param->uh_param);
	if (ret)
		goto ul_error;

	ret = no_os_gpio_direction_output(descriptor->wvu_desc[5], NO_OS_GPIO_LOW);
	if (ret)
		goto uh_error;

	ret = no_os_gpio_get_optional(descriptor->spe_desc, init_param->spe_param);
	if (ret)
		goto uh_error;

	if (descriptor->spe_desc) {
		ret = no_os_gpio_direction_output(descriptor->spe_desc, NO_OS_GPIO_HIGH);
		if (ret)
			goto spe_error;
	}

	ret = no_os_gpio_get(descriptor->drv_en_desc, init_param->drv_en_param);
	if (ret)
		goto spe_error;

	ret = no_os_gpio_direction_output(descriptor->drv_en_desc, NO_OS_GPIO_HIGH);
	if (ret)
		goto drv_en_error;

	*desc = descriptor;

	return 0;

drv_en_error:
	no_os_gpio_remove(descriptor->drv_en_desc);
spe_error:
	no_os_gpio_remove(descriptor->spe_desc);
uh_error:
	no_os_gpio_remove(descriptor->wvu_desc[5]);
ul_error:
	no_os_gpio_remove(descriptor->wvu_desc[4]);
vh_error:
	no_os_gpio_remove(descriptor->wvu_desc[3]);
vl_error:
	no_os_gpio_remove(descriptor->wvu_desc[2]);
wh_error:
	no_os_gpio_remove(descriptor->wvu_desc[1]);
wl_error:
	no_os_gpio_remove(descriptor->wvu_desc[0]);
spi_error:
	no_os_spi_remove(descriptor->comm_desc);
free_tmc6100:
	no_os_free(descriptor);

	return ret;
}

int tmc6100_remove(struct tmc6100_desc *desc)
{
	int i;

	for (i = 0; i < 6; i++)
		no_os_gpio_set_value(desc->wvu_desc[i], NO_OS_GPIO_LOW);

	no_os_gpio_remove(desc->drv_en_desc);
	no_os_gpio_remove(desc->spe_desc);

	for (i = 0; i < 6; i++)
		no_os_gpio_remove(desc->wvu_desc[i]);

	no_os_spi_remove(desc->comm_desc);
	no_os_free(desc);

	return 0;
}
