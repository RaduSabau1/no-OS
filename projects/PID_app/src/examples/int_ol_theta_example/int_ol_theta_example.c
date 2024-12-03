#include "int_ol_theta_example.h"
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
#define QBL4208_RATED_SPEED		4000.0f
#define QBL4208_RATED_TORQUE		0.125f
#define QBL4208_RATED_VOLTAGE		24
#define QBL4208_RATED_PHASE_CURRENT	3.47
#define QBL4208_RATED_POWER		(QBL4208_RATED_VOLTAGE * QBL4208_RATED_PHASE_CURRENT)
#define QBL4208_TORQUE_CONSTANT		0.036

/* Unit Conversions. */
#define PID_RPM_TO_NM(x)		(((x) * 0.125) / QBL4208_RATED_SPEED)
#define PID_NM_TO_A(x)			((x) / QBL4208_TORQUE_CONSTANT)

#define UQ_SCALE			2.1
#define UD_SCALE			2.1

#define DEC_TO_DEG(x)			(((x) * 360) / 65535)
#define DEG_TO_RAD(x)			((x) * 0.0174532925)

int int_ol_theta_example_main()
{
	struct tmc6100_desc *tmc6100_desc;
	struct tmc4671_desc *tmc4671_desc;

	struct sIPark ptIPark = {
		.fD = 0.0, /* Rs = 24 / 3.47 = 6.91, Vd = Id * Rs, Id = 0.14A */
		.fQ = 2.0, /* Rs = 24 / 3.47 = 6.91, Vq = Iq * Rs, Iq = 0.14A */
	};
	struct sSVPWM tSVPWM = {
		.enInType = AlBe,
		.fUdc = 24, // 24V Max DC-Link Voltage.
		.fUdcCCRval = PWM_SECTOR_PERIOD_NS, /* 18000ns = 18us = 90% tPWM (20us => 50kHz)*/
	};

	struct sFFClarke ptFFClarke;
	struct sFPark ptFPark;

	struct sPI speed_pi = {
		.fDtMicroSec = 10,
		.fKp = 1,
		.fKi = 1,
		.fUpOutLim = 32000,
		.fLowOutLim = -32000,
	};
	struct sP torque_pi = {
		.fKp = 1,
		.fUpOutLim = 32767.0f,
		.fLowOutLim = -32767.0f,
	};
	struct sP flux_pi = {
		.fKp = 1,
		.fUpOutLim = 8192,
		.fLowOutLim = -8192,
	};

	uint32_t angle, duty_cycle, reg_val, meas_iq_id = 0, offset1, offset0;
	uint32_t uns_measiq, uns_measid;
	int16_t adcu, adcw;
	int16_t meas_iq = 0, meas_id = 0;
	int32_t ref_speed = 1000; /* Electrical Speed reference. */
	int32_t error_speed = 1;
	int32_t meas_speed = 0;
	float elec_angle;
	float ref_torque;
	int ret, i;

	int min = 5000, max = 0;
	float minq = 10, maxq = -10;
	float iu, iv, iw;

	bool open_loop = true;

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

	/* Start QBL4208 motor. */
	ret = tmc6100_start_bldc_motor(tmc6100_desc);
	if (ret)
		goto remove_tmc4671;

	if (open_loop) {
		/* Open-Loop operation. */
		while (1) {
			ret = tmc4671_reg_read(tmc4671_desc, TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E_REG,
					       &angle);
			if (ret)
				goto stop_motor;

			angle &= 0xFFFF;

			elec_angle = (float)angle;

			elec_angle = DEC_TO_DEG(elec_angle);
			elec_angle = DEG_TO_RAD(elec_angle);

			/* Read actual Q, D current values. */
			ret = tmc4671_reg_read(tmc4671_desc, 0x6E, &reg_val);
			if (ret)
				goto stop_motor;

			meas_iq = (int16_t)(reg_val & 0xFFFF);

			pr_info("Avg Q : 0x%x\n", meas_iq);

			ptIPark.fSinAng = sinf(elec_angle);
			ptIPark.fCosAng = cosf(elec_angle);

			tIPark_dq2albe(&ptIPark);
			tSVPWM.fUal = ptIPark.fAl;
			tSVPWM.fUbe = ptIPark.fBe;
			tSVPWM_calc(&tSVPWM);

			ret = tmc6100_set_duty(tmc6100_desc,
					       (uint32_t)tSVPWM.fCCRA,
					       (uint32_t)tSVPWM.fCCRB,
					       (uint32_t)tSVPWM.fCCRC);
			if (ret)
				goto stop_motor;
		}
	} else {
		/* Closed Loop Operation.*/
		while (1) {
			/* Theta (Electric Angle) computation. */
			ret = tmc4671_read_angle(tmc4671_desc, &angle);
			if (ret)
				goto stop_motor;

			elec_angle = DEG_TO_RAD(DEC_TO_DEG((float)angle));

			ret = tmc4671_reg_read(tmc4671_desc, TMC4671_ADC_IWY_IUX_REG, &reg_val);
			if (ret)
				goto stop_motor;

			iu = (float)(int16_t)(reg_val & 0xFFFF);
			iw = (float)(int16_t)((reg_val >> 16) & 0xFFFF);
			iu *= 0.00066187f;
			iv *= 0.00066187f;
			iv = (iu + iw) * (-1);

			ptFFClarke.fA = iu;
			ptFFClarke.fB = iv;
			ptFFClarke.fC = iw;

			tFFClarke_abc2albe(&ptFFClarke);

			ptFPark.fAl = ptFFClarke.fAl;
			ptFPark.fBe = ptFFClarke.fBe;
			ptFPark.fCosAng = cosf(elec_angle);
			ptFPark.fSinAng = sinf(elec_angle);

			tFPark_albe2dq(&ptFPark);

			ret = tmc4671_read_velocity(tmc4671_desc, &meas_speed);
			if (ret)
				return ret;

			/* Electrical Speed -> Mechanical Speed */
			pr_info("Speed : %ld RPM\n", meas_speed / 4);

			/* Speed PI Controller. */
			speed_pi.fIn = ref_speed - meas_speed;

			tPI_calc(&speed_pi);

			/* Speed to Torque (RPM to A) reference for Torque PI.*/
			ref_torque = PID_RPM_TO_NM(PID_NM_TO_A(speed_pi.fOut));

			pr_info("Meas Q : %0.5f\n", ptFPark.fQ);
			pr_info("Ref Q : %0.5f\n", ref_torque);
			torque_pi.fIn = ref_torque - ptFPark.fQ;
			pr_info("Error Q : %0.5f\n", torque_pi.fIn);
			flux_pi.fIn = 0 - ptFPark.fD;

			/* Flux PI Controller. */
			tP_calc(&flux_pi);
			/* Torque PI Controller. */
			tP_calc(&torque_pi);

			ptIPark.fD = flux_pi.fOut;
			ptIPark.fQ = torque_pi.fOut;

			pr_info("Q : %0.5f\n", torque_pi.fOut);
			pr_info("D : %0.5f\n", flux_pi.fOut);

			/* Inside Loop Operation. */
			ptIPark.fCosAng = cosf(elec_angle);
			ptIPark.fSinAng = sinf(elec_angle);

			tIPark_dq2albe(&ptIPark);
			tSVPWM.fUal = ptIPark.fAl;
			tSVPWM.fUbe = ptIPark.fBe;
			tSVPWM_calc(&tSVPWM);

			ret = tmc6100_set_duty(tmc6100_desc,
					       (uint32_t)tSVPWM.fCCRA,
					       (uint32_t)tSVPWM.fCCRB,
					       (uint32_t)tSVPWM.fCCRC);
			if (ret)
				goto stop_motor;
		}
	}

stop_motor:
	tmc6100_stop_bldc_motor(tmc6100_desc);
remove_tmc4671:
	tmc4671_remove(tmc4671_desc);
remove_tmc6100:
	tmc6100_remove(tmc6100_desc);
exit:
	if (ret)
		pr_info("Error!\n");
	return ret;
}
