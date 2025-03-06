#include "ext_loop_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "tmc4671.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_pwm.h"
#include "vector_transfs.h"
#include "fp_pid.h"
#include "no_os_units.h"

#define RATED_SPEED			12850

/* Unit Conversions. */
#define RPM_TO_NQ(x)			(((x) * 0x7FFF) / RATED_SPEED)

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
	int16_t uq;

	int speed_stamp[1000] = {0};

	float kp_speed, ki_speed, kp_current, ki_current;
	kp_speed =1;
	ki_speed =1;
	kp_current =1;
	ki_current =1;
	ref_speed =1000;
	struct sPI speed_pi = {
		.fDtSec = 0.001000f,
		.fKp = kp_speed,
		.fKi = ki_speed,
		.fUpOutLim = 7000, /* Max Peak Torque Requirement. */
		.fLowOutLim = -7000, /* Min Peak Torque Requirement. */
	};
	struct sPI current_pi = {
		.fDtSec = 0.000500f,
		.fKp = kp_current,
		.fKi = ki_current,
		.fUpOutLim = 32767, /* Max Peak Torque Requirement. */
		.fLowOutLim = -32767, /* Min Peak Torque Requirement. */
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

	ret = tmc4671_phi_e_sel(tmc4671_desc, TMC4671_PHI_E_AENC);
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

	/* Open Loop Operation. */
	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0x30000000);
	if (ret)
		goto remove_tmc4671;

	/* Closed Loop Operation.*/
	// while (i < 10000) {
	// 	/* Outer Loop (Speed). */
	// 	ret = tmc4671_read_velocity(tmc4671_desc, &meas_speed);
	// 	if (ret)
	// 		return ret;

	// 	/* Inner Loop (Current). */
	// 	ret = tmc4671_reg_read(tmc4671_desc, TMC4671_PID_TORQUE_FLUX_ACTUAL_REG, &reg_val);
	// 	if (ret)
	// 		return ret;

	// 	/* Speed PI Controller. */
	// 	speed_pi.fIn = ref_speed - meas_speed;

	// 	tPI_calc(&speed_pi);

	// 	/* Current PI Controller. */
	// 	current_pi.fIn = RPM_TO_NQ(speed_pi.fOut) - no_os_sign_extend32(((reg_val >> 16) & 0xFFFF), 15);

	// 	tPI_calc(&current_pi);

	// 	uq = (int16_t)RPM_TO_NQ(speed_pi.fOut);
	// 	uq_ud = (((uint32_t)uq << 16) & 0xFFFF0000);

	// 	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, uq_ud);
	// 	if (ret)
	// 		goto stop_motor;

	// 	if (i % 10 == 0) {
	// 		j = i / 10;
	// 		speed_stamp[j] = meas_speed;
	// 	}

	// 	i++;
	// }

	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0);
	if (ret)
		goto stop_en;

	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_OFF);

	for (j = 0; j < 1000; j++)
		pr_info("%i %i %i %i\n", j, speed_stamp[j]);

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
