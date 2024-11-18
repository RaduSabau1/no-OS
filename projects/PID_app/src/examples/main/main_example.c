#include "main_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "tmc4671.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_pwm.h"
#include "svpwm.h"
#include "vector_transfs.h"
#include "im_estimators.h"
#include "fp_pid.h"
#include "no_os_units.h"

#define KP			5
#define KI			1
#define QBL4208_RATED_SPEED	4000

/* Use TMC4671 vector control for Open-Loop control. */
// int main_example_main()
// {
// 	struct tmc6100_desc *tmc6100_desc;
// 	struct tmc4671_desc *tmc4671_desc;
// 	uint8_t tmc6100_sequence_1[6];
// 	uint8_t tmc6100_sequence_2[6];
// 	uint32_t angle, duty_cycle;
// 	int32_t ref_speed = 1500;
// 	int32_t meas_speed;
// 	int ret, i;

// 	int32_t error_speed = 1, y1;
// 	uint32_t ud = 0x05000500;

// 	/* Used for test. */
// 	struct no_os_gpio_desc *trig_gpio;
// 	struct no_os_gpio_init_param trig_gpio_param = {
// 		.port = 1,
// 		.pull = NO_OS_PULL_NONE,
// 		.number = 1,
// 		.platform_ops = GPIO_OPS,
// 		.extra = GPIO_EXTRA
// 	};

// 	ret = no_os_gpio_get(&trig_gpio, &trig_gpio_param);
// 	if (ret)
// 		goto exit;

// 	ret = no_os_gpio_direction_output(trig_gpio, NO_OS_GPIO_HIGH);
// 	if (ret)
// 		goto remove_trig;

// 	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
// 	if (ret)
// 		goto remove_trig;

// 	ret = tmc4671_init(&tmc4671_desc, &tmc4671_ip);
// 	if (ret)
// 		goto remove_tmc6100;

// 	ret = tmc6100_reg_update(tmc6100_desc, TMC6100_GCONF_REG, NO_OS_BIT(6), 1);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc4671_phi_e_sel(tmc4671_desc, TMC4671_PHI_E_HAL);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc4671_set_pwm_freq(tmc4671_desc, 50000);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc4671_set_pwm_bbm(tmc4671_desc, 0);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_CENTER);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc4671_set_mode_motion(tmc4671_desc, TMC4671_UQ_UD_EXT);
// 	if (ret)
// 		goto remove_tmc4671;

// 	/* Open-Loop operation. */
// 	no_os_gpio_set_value(trig_gpio, NO_OS_GPIO_LOW);
// 	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, ud);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc4671_reg_write(tmc4671_desc, TMC4671_UQ_UD_EXT_REG, 0);
// 	if (ret)
// 		goto remove_tmc4671;

// remove_tmc4671:
// 	tmc4671_remove(tmc4671_desc);
// remove_tmc6100:
// 	tmc6100_remove(tmc6100_desc);
// remove_trig:
// 	no_os_gpio_remove(trig_gpio);
// exit:
// 	if (ret)
// 		pr_info("Error!\n");
// 	return ret;
// }

/* Use STM32 with Angle Reading from TMC4671 for Open-Loop Control. */
// int main_example_main()
// {
// 	struct tmc6100_desc *tmc6100_desc;
// 	struct tmc4671_desc *tmc4671_desc;
// 	uint8_t tmc6100_sequence_1[6];
// 	uint8_t tmc6100_sequence_2[6];
// 	uint32_t angle, duty_cycle, ud = 0;
// 	int32_t ref_speed = 1500;
// 	struct sIPark ptIPark = {
// 		.fD = 0.9674, /* Rs = 24 / 3.47 = 6.91, Vd = Id * Rs, Id = 0.14A */
// 		.fQ = 0.9674, /* Rs = 24 / 3.47 = 6.91, Vq = Iq * Rs, Iq = 0.14A */
// 	};
// 	struct sSVPWM tSVPWM = {
// 		.enInType = AlBe,
// 		.fUdc = 41.52f,
// 		.fUdcCCRval = 20000,
// 	};
// 	int32_t meas_speed;
// 	float real_angle;
// 	int ret, i;

// 	int32_t error_speed = 1, y1;

// 	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
// 	if (ret)
// 		goto exit;

// 	ret = tmc4671_init(&tmc4671_desc, &tmc4671_ip);
// 	if (ret)
// 		goto remove_tmc6100;

// 	ret = tmc6100_reg_update(tmc6100_desc, TMC6100_GCONF_REG, NO_OS_BIT(6), 1);
// 	if (ret)
// 		goto remove_tmc4671;

// 	ret = tmc6100_start_bldc_motor(tmc6100_desc);
// 	if (ret)
// 		goto remove_tmc4671;

// 	/* Open-Loop operation. */
// 	while (1) {
// 		ret = tmc4671_reg_read(tmc4671_desc, TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E_REG, &angle);
// 		if (ret)
// 			goto remove_tmc4671;


// 		/* Electrical Angle to radians (taking pole number into consideration). */
// 		real_angle = ((float)angle / (tmc4671_desc->no_pole_pairs * 5461.0f));
// 		ptIPark.fSinAng = sinf(real_angle);
// 		ptIPark.fCosAng = cosf(real_angle);

// 		tIPark_dq2albe(&ptIPark);
// 		tSVPWM.fUal = ptIPark.fAl;
// 		tSVPWM.fUbe = ptIPark.fBe;
// 		tSVPWM_calc(&tSVPWM);

// 		ret = tmc6100_set_duty(tmc6100_desc, (uint32_t)tSVPWM.fCCRA,
// 				       (uint32_t)tSVPWM.fCCRB,
// 				       (uint32_t)tSVPWM.fCCRC);
// 		if (ret)
// 			goto remove_tmc4671;

// 		i++;
// 	}

// remove_tmc4671:
// 	tmc4671_remove(tmc4671_desc);
// remove_tmc6100:
// 	tmc6100_remove(tmc6100_desc);
// exit:
// 	if (ret)
// 		pr_info("Error!\n");
// 	return ret;
// }

/* Use STM32 with sensorless Angle Estimator for Open-Loop Control. */
int main_example_main()
{
	struct tmc6100_desc *tmc6100_desc;
	struct tmc4671_desc *tmc4671_desc;
	uint8_t tmc6100_sequence_1[6];
	uint8_t tmc6100_sequence_2[6];
	int32_t ref_speed = 1500;
	uint32_t duty_cycle;
	int32_t meas_speed;
	float angle = 0;
	int ret, i;

	int32_t error_speed = 1, y1;
	uint32_t ud = 0x05000500;

	struct sIPark ptIPark_I = {
		.fD = 0.14, /* Rs = 24 / 3.47 = 6.91, Vd = Id * Rs, Id = 0.14A */
		.fQ = 0.14, /* Rs = 24 / 3.47 = 6.91, Vq = Iq * Rs, Iq = 0.14A */
	};
	struct sSVPWM tSVPWM = {
		.enInType = AlBe,
		.fUdc = 41.52f,
		.fUdcCCRval = 20000,
	};
	struct sIMparams tIMparams = {
		.fDt = 0.000002,		// Discretization time, Sec
		.fNpP = 4,			// Count of pole pairs
		.fRs = 0.72,			// Stator resistance, Ohm
		.fRr = 6.91,			// Rotor resistance, Ohm
		.fLs = 0.0012,			// Stator inductance, H
		.fLr = 0.0006,			// Rotor inductance, H
		.fLm = 0.0001,			// Magnetizing inductance, H
		.f1divTr = 0,		// fRr/fLr
		.f1divKr = 0,			// fLr/fLm
		.fSigLs = 0,			// (1 - (fLm^2)/(fLs*fLr)) * fLs
	};
	struct sIMrotObs tIMrotObs = {
		.fIsAl = 0,			// Stator current Alpha, A
		.fIsBe = 0,			// Stator current Beta, A
		.fWrE = 0,			// Rotor electrical speed, Rad/Sec
		.fPrevErAl = 0,			// Previous value of rotor back-EMF
		// Alpha, Volts
		.fPrevErBe = 0,			// Previous value of rotor back-EMF
		// Beta, Volts
		.fPrevFrAl = 0,			// Previous value of rotor flux
		// Alpha, Wb
		.fPrevFrBe = 0,			// Previous value of rotor flux
		// Beta, Wb
		.fFrAl = 0,			// Rotor flux Alpha, Wb
		.fFrBe = 0,			// Rotor flux Beta, Wb
		.fErAl = 0,			// Rotor back-EMF Alpha, Volts
		.fErBe = 0,			// Rotor back-EMF Beta, Volts
	};
	tIMparams_init(&tIMparams);

	struct no_os_gpio_init_param trig_gpio_ip = {
		.port = 2,
		.pull = NO_OS_PULL_NONE,
		.number = 8,
		.platform_ops = GPIO_OPS,
		.extra = GPIO_EXTRA
	};
	struct no_os_gpio_desc *trig_gpio;

	ret = no_os_gpio_get(&trig_gpio, &trig_gpio_ip);
	if (ret)
		goto exit;

	ret = no_os_gpio_direction_output(trig_gpio, NO_OS_GPIO_HIGH);
	if (ret)
		goto remove_trig;

	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
	if (ret)
		goto remove_trig;

	ret = tmc4671_init(&tmc4671_desc, &tmc4671_ip);
	if (ret)
		goto remove_tmc6100;

	ret = tmc6100_reg_update(tmc6100_desc, TMC6100_GCONF_REG, NO_OS_BIT(6), 1);
	if (ret)
		goto remove_tmc4671;

	ret = tmc6100_start_bldc_motor(tmc6100_desc);
	if (ret)
		goto remove_tmc4671;

	no_os_gpio_set_value(trig_gpio, NO_OS_GPIO_LOW);
	ret = tmc6100_set_duty(tmc6100_desc, 9000, 11000, 10000);
	if (ret)
		goto remove_tmc4671;

	/* Rest of sensorless implementation. */
	while (1) {
		ret = tmc4671_reg_read(tmc4671_desc,
				       TMC4671_PID_VELOCITY_ACTUAL_REG,
				       &meas_speed);
		if (ret)
			goto remove_tmc4671;

		ptIPark_I.fSinAng = sinf(angle);
		ptIPark_I.fCosAng = cosf(angle);
		tIPark_dq2albe(&ptIPark_I);
		tSVPWM.fUal = ptIPark_I.fAl;
		tSVPWM.fUbe = ptIPark_I.fBe;
		tSVPWM_calc(&tSVPWM);

		ret = tmc6100_set_duty(tmc6100_desc, (uint32_t)tSVPWM.fCCRA,
				       (uint32_t)tSVPWM.fCCRB,
				       (uint32_t)tSVPWM.fCCRC);
		if (ret)
			goto remove_tmc4671;

		tIMrotObs.fIsAl = ptIPark_I.fAl;
		tIMrotObs.fIsBe = ptIPark_I.fBe;
		tIMrotObs.fWrE = meas_speed;
		tIMrotObs_calc(&tIMrotObs, &tIMparams);

		angle = atan2f(tIMrotObs.fFrAl, tIMrotObs.fFrBe);

		no_os_udelay(tIMparams.fDt * MICRO);
	}

remove_tmc4671:
	tmc4671_remove(tmc4671_desc);
remove_tmc6100:
	tmc6100_remove(tmc6100_desc);
remove_trig:
	no_os_gpio_remove(trig_gpio);
exit:
	if (ret)
		pr_info("Error!\n");
	return ret;
}
