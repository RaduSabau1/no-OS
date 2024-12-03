#include "ext_ol_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "tmc4671.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_pwm.h"
#include "svpwm.h"
#include "vector_transfs.h"
#include "fp_pid.h"
#include "no_os_units.h"

/* Motor Specifications. */
#define QBL4208_RATED_SPEED		4000
#define QBL4208_RATED_TORQUE		0.125
#define QBL4208_RATED_VOLTAGE		24
#define QBL4208_RATED_PHASE_CURRENT	3.47
#define QBL4208_RATED_POWER		(QBL4208_RATED_VOLTAGE * QBL4208_RATED_PHASE_CURRENT)
#define QBL4208_TORQUE_CONSTANT		0.036

/* Unit Conversions. */
#define RPM_TO_N(x, y)			(((x) * 23) / 10 * (y))
#define FULL_TIME(x, y)			(((x) * MILLI) / y)

#define DEC_TO_DEG(x)			(((x) * 360) / 65535)
#define DEG_TO_RAD(x)			((x) * 0.0174532925)

/* Use TMC4671 vector control for Open-Loop control. */
int ext_ol_example_main()
{
	struct tmc6100_desc *tmc6100_desc;
	struct tmc4671_desc *tmc4671_desc;

	struct sFFClarke ptFFClarke;
	struct sFPark ptFPark;

	int kp_speed, ki_speed, kp_torque, kp_flux;
	kp_speed = 1;
	ki_speed = 1;
	kp_torque = 1;
	kp_flux = 1;
	struct sPI speed_pi = {
		.fDtMicroSec = 100,
		.fKp = kp_speed,
		.fKi = ki_speed,
		.fUpOutLim = 4000,
		.fLowOutLim = -4000,
	};
	struct sP torque_pi = {
		.fKp = kp_torque,
		.fUpOutLim = 32767,
		.fLowOutLim = -32767,
	};
	struct sP flux_pi = {
		.fKp = kp_flux,
		.fUpOutLim = 8192,
		.fLowOutLim = -8192,
	};

	int32_t ref_speed, meas_speed = 0;
	uint32_t reg_val, angle, uq_ud;
	int i = 0, j = 0, k = 0, ret;
	int16_t ud, uq;

	int speed_stamp[1000] = {0};
	int torque_stamp[1000] = {0};
	int flux_stamp[1000] = {0};

	bool open_loop = false;

	/* = No. Pole-Pairs * Mechanical Speed Reference. => Electrical Speed Reference*/
	ref_speed = 2000;

	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
	if (ret)
		goto exit;

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

	if (open_loop) {
		uq_ud = 0x10000000;
		/* Open-Loop operation. */
		ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG,
					uq_ud);
		if (ret)
			goto stop_motor;

		no_os_mdelay(1000);

		uq_ud = 0x10001000;
		ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG,
					uq_ud);
		if (ret)
			goto stop_motor;

		no_os_mdelay(1000);
	} else {
		/* Closed Loop Operation.*/
		while (i < 10000) {
			/* Outer Loop (Speed). */
			ret = tmc4671_read_velocity(tmc4671_desc, &meas_speed);
			if (ret)
				return ret;

			/* Speed PI Controller. */
			speed_pi.fIn = ref_speed - meas_speed;

			tPI_calc(&speed_pi);

			/* Inside Loop (Torque and Flux). */
			ret = tmc4671_reg_read(tmc4671_desc, TMC4671_PID_TORQUE_FLUX_ACTUAL_REG,
					       &reg_val);
			if (ret)
				goto stop_motor;

			uq = (int16_t)((reg_val >> 16) & 0xFFFF);
			ud = (int16_t)(reg_val & 0xFFFF);

			torque_pi.fIn = RPM_TO_N(speed_pi.fOut, kp_speed) - uq;
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
	}

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0);
	if (ret)
		goto stop_en;

	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_OFF);

	if (!open_loop) {
		for (j = 0; j < 1000; j++) {
			pr_info("%f %i %f %f\n", (float)j / 1000.0f, speed_stamp[j],
				(float)torque_stamp[j] * 0.000661887, (float)flux_stamp[j] * 0.00066187);
		}
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
