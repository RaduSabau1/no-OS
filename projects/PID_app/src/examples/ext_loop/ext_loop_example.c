#include "ext_loop_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "tmc4671.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "fp_pid.h"
#include "no_os_units.h"
#include "maxim_spi.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* Unit Conversions. */
#define RPM_TO_NQ(x)			((x) * 10)

// int reg_read(struct tmc4671_desc *desc, uint8_t reg, uint32_t *val)
// {
// 	struct no_os_spi_msg xfer = {
// 		.tx_buff = desc->buff,
// 		.rx_buff = desc->buff,
// 		.bytes_number = 5,
// 		.cs_change = 1,
// 	};
// 	int ret;

// 	desc->buff[0] = reg;

// 	ret = max_spi_transfer(desc->spi_desc, &xfer, 1);
// 	if (ret)
// 		return ret;

// 	*val = no_os_get_unaligned_be32(&desc->buff[1]);

// 	return 0;
// }

// int reg_write(struct tmc4671_desc *desc, uint8_t reg, uint32_t val)
// {
// 	struct no_os_spi_msg xfer = {
// 		.tx_buff = desc->buff,
// 		.rx_buff = desc->buff,
// 		.bytes_number = 5,
// 		.cs_change = 1,
// 	};

// 	desc->buff[0] = reg | TMC4671_RW_MASK;
// 	no_os_put_unaligned_be32(val, &desc->buff[1]);

// 	return max_spi_transfer(desc->spi_desc, &xfer, 1);
// }

/**
 * @brief Speed, Torque and Flux Loops are performed externally by the host.
 * @return 0 in case of success, negative error code otherwise.
 */
int ext_loop_example_main()
{
	struct tmc6100_desc *tmc6100_desc;
	struct tmc4671_desc *tmc4671_desc;

	int32_t ref_speed, meas_speed = 0;
	uint32_t reg_val, uq_ud;
	int i = 0, j = 0, ret;
	int16_t ud, uq;

	int speed_stamp[1000] = {0};
	int torque_stamp[1000] = {0};
	int flux_stamp[1000] = {0};

	float kp_speed, ki_speed, kp_torque, kp_flux;
<<<<<<< HEAD
	kp_speed =1.04;
	ki_speed =0.006;
	kp_torque =0.72;
	kp_flux =0.72;
	ref_speed =4000;
=======
	kp_speed =1.5;
	ki_speed =1.4;
	kp_torque =1.1;
	kp_flux =1.7;
	ref_speed =745;
>>>>>>> f4da27a86 (projects: PID_app : Add Hardware-In-The-Loop support with Simulink)
	struct sPI speed_pi = {
		.fDtSec = 0.000500f,
		.fKp = kp_speed,
		.fKi = ki_speed,
		.fUpOutLim = 7000, /* Max Peak Torque Requirement. */
		.fLowOutLim = -7000, /* Min Peak Torque Requirement. */
	};
	struct sP torque_pi = {
		.fKp = kp_torque,
		.fUpOutLim = 32767,
		.fLowOutLim = -32767,
	};
	struct sP flux_pi = {
		.fKp = kp_flux,
		.fUpOutLim = 32767,
		.fLowOutLim = -32767,
	};

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
				0x00108001);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_ADC_I0_SCALE_OFFSET_REG,
				0x00108001);
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

	/* Start Motor. */
	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0x0);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_CENTER);
	if (ret)
		goto remove_tmc4671;

	/* Closed Loop Operation.*/
	while (i < 10000) {
		/* Outer Loop (Speed). */

		ret = tmc4671_reg_read(tmc4671_desc, TMC4671_PID_VELOCITY_ACTUAL_REG, &reg_val);
		if (ret)
			return ret;

		meas_speed = (int32_t)reg_val;

		/* Inside Loop (Torque and Flux). */
		ret = tmc4671_reg_read(tmc4671_desc, TMC4671_PID_TORQUE_FLUX_ACTUAL_REG,
				       &reg_val);
		if (ret)
			goto stop_motor;

		uq = (int16_t)((reg_val >> 16) & 0xFFFF);
		ud = (int16_t)(reg_val & 0xFFFF);

		/* Speed PI Controller. */
		speed_pi.fIn = ref_speed - meas_speed;
		tPI_calc(&speed_pi);

		torque_pi.fIn = RPM_TO_NQ(speed_pi.fOut) - uq;
		flux_pi.fIn = 0 - ud;

		/* Flux PI Controller. */
		tP_calc(&flux_pi);
		/* Torque PI Controller. */
		tP_calc(&torque_pi);

		uq = (int16_t)(torque_pi.fOut);
		ud = (int16_t)(flux_pi.fOut);

		uq_ud = (((uint32_t)uq << 16) & 0xFFFF0000) | ((uint32_t)ud & 0xFFFF);

		ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, uq_ud);
		if (ret)
			goto stop_motor;

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

	for (j = 0; j < 1000; j++)
		pr_info("%i %i %i %i\n", j, speed_stamp[j], torque_stamp[j],
			flux_stamp[j]);

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
