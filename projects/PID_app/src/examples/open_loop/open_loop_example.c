#include "open_loop_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "tmc4671.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_pwm.h"
#include "no_os_units.h"

/**
 * @brief Torque and Flux run in open loop mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int open_loop_example_main()
{
	struct tmc6100_desc *tmc6100_desc;
	struct tmc4671_desc *tmc4671_desc;

	uint32_t reg_val, uq_ud;
	int i = 0, j = 0, ret;
	int32_t meas_speed;
	int16_t uq, ud;

	float torque, flux;

	int speed_stamp[1000] = {0};
	int torque_stamp[1000] = {0};
	int flux_stamp[1000] = {0};

	torque = 1;
	flux = 0.3;

	/* TMC6100 CFG. */
	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
	if (ret)
		goto exit;

	/* TMC4671 CFG. */
	ret = tmc4671_init(&tmc4671_desc, &tmc4671_ip);
	if (ret)
		goto remove_tmc6100;

	ret = tmc6100_reg_update(tmc6100_desc, TMC6100_GCONF_REG, NO_OS_BIT(6), 1);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_ADC_I_SELECT_REG, 0x09000100);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_ADC_I1_SCALE_OFFSET_REG,
				0x00408001);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_ADC_I0_SCALE_OFFSET_REG,
				0x00408001);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_phi_e_sel(tmc4671_desc, TMC4671_PHI_E_HAL);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_pwm_freq(tmc4671_desc, 50000);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_pwm_bbm(tmc4671_desc, 5);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_PWM_POLARITIES_REG, 0x03);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_mode_motion(tmc4671_desc, TMC4671_UQ_UD_EXT);
	if (ret)
		goto remove_tmc4671;

	/* Stop Motor. */
	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_CENTER);
	if (ret)
		goto remove_tmc4671;

	/* Open Loop Operation.*/
	uq = (int16_t)(torque / 0.00066187f);
	ud = (int16_t)(flux / 0.00066187f);
	uq_ud = (((uint32_t)uq << 16) & 0xFFFF0000) | ((uint32_t)ud & 0xFFFF);
	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG,
				uq_ud);
	if (ret)
		goto stop_motor;

	i = 0;
	while (i < 10000) {
		ret = tmc4671_read_velocity(tmc4671_desc, &meas_speed);
		if (ret)
			goto stop_motor;

		ret = tmc4671_reg_read(tmc4671_desc, TMC4671_PID_TORQUE_FLUX_ACTUAL_REG,
				       &reg_val);
		if (ret)
			goto stop_motor;

		uq = ((int16_t)reg_val >> 16) & 0xFFFF;
		ud = (int16_t)reg_val & 0xFFFF;

		if (i % 10 == 0) {
			j = i / 10;
			speed_stamp[j] = meas_speed;
			torque_stamp[j] = uq;
			flux_stamp[j] = ud;
		}
		i++;
	}

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0);
	if (ret)
		goto stop_en;

	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_OFF);

	for (j = 0; j < 1000; j++) {
		pr_info("%f %i %f %f\n", (float)j / 1000.0f, speed_stamp[j],
			(float)torque_stamp[j] * 0.000661887, (float)flux_stamp[j] * 0.00066187);
	}

	goto stop_en;

stop_motor:
	tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0);
	tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_OFF);
stop_en:
	no_os_gpio_set_value(tmc6100_desc->drv_en_desc, NO_OS_GPIO_LOW);
remove_tmc4671:
	tmc4671_remove(tmc4671_desc);
remove_tmc6100:
	tmc6100_remove(tmc6100_desc);
exit:
	if (ret)
		pr_info("Error!\n");
	return ret;
}
