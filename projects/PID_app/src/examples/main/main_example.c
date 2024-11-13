#include "main_example.h"
#include "common_data.h"
#include "tmc6100.h"
#include "tmc4671.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_pwm.h"

#define KP			5
#define QBL4208_RATED_SPEED	4000

int main_example_main()
{
	// struct no_os_pwm_desc *pwm_sector_sel;
	// struct no_os_gpio_desc *pwmin_gpio;
	struct tmc6100_desc *tmc6100_desc;
	struct tmc4671_desc *tmc4671_desc;
	enum tmc6100_bldc_sector sector;
	uint8_t tmc6100_sequence_1[6];
	uint8_t tmc6100_sequence_2[6];
	uint32_t angle, duty_cycle, ud = 0;
	int32_t ref_speed = 1500;
	int32_t meas_speed;
	uint8_t pwm_state;
	int ret;

	int32_t error_speed = 1, y1;

	ret = tmc6100_init(&tmc6100_desc, &tmc6100_ip);
	if (ret)
		goto exit;

	// ret = tmc6100_bldc_create_seq(tmc6100_desc, TMC6100_UH,
	// 			      tmc6100_sequence_1);
	// if (ret)
	// 	goto remove_tmc6100;

	// ret = tmc6100_bldc_create_seq(tmc6100_desc, TMC6100_UL,
	// 			      tmc6100_sequence_2);
	// if (ret)
	// 	goto remove_tmc6100;

	ret = tmc4671_init(&tmc4671_desc, &tmc4671_ip);
	if (ret)
		goto remove_tmc6100;

	ret = tmc4671_phi_e_sel(tmc4671_desc, TMC4671_PHI_E_HAL);
	if (ret)
		goto remove_tmc4671;

	ret = tmc6100_reg_update(tmc6100_desc, TMC6100_GCONF_REG, NO_OS_BIT(6), 1);
	if (ret)
		goto remove_tmc4671;

	// ret = tmc6100_reg_update(tmc6100_desc, TMC6100_DRV_CONF_REG, 0x03, 2);
	// if (ret)
	// 	goto remove_tmc4671;

	// ret = tmc6100_start_bldc_motor(tmc6100_desc);
	// if (ret)
	// 	goto remove_tmc4671;

	// ret = no_os_pwm_init(&pwm_sector_sel, &pwm_sector_ip);
	// if (ret)
	// 	goto remove_tmc4671;

	// ret = no_os_gpio_get(&pwmin_gpio, &pwmin_gpio_ip);
	// if (ret)
	// 	goto remove_pwm_sector;

	// ret = no_os_gpio_direction_input(pwmin_gpio);
	// if (ret)
	// 	goto remove_pwmin_gpio;

	ret = tmc4671_set_pwm_freq(tmc4671_desc, 25000);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_pwm_bbm(tmc4671_desc, 5);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_pwm_sv_chop(tmc4671_desc, TMC4671_PWM_CENTER);
	if (ret)
		goto remove_tmc4671;

	ret = tmc4671_set_mode_motion(tmc4671_desc, TMC4671_UQ_UD_EXT);
	if (ret)
		goto remove_tmc4671;

	if (ref_speed > QBL4208_RATED_SPEED)
		goto remove_tmc4671;

	// ret = tmc6100_bldc_increase_speed(tmc6100_desc, 30000);
	// if (ret)
	// 	goto remove_tmc4671;



	/* Actual Motor control sequence. */
	while (error_speed != 0) {
		// ret = tmc4671_read_angle(tmc4671_desc, &angle);
		// if (ret)
		// 	goto remove_pwmin_gpio;

		// if (angle == 360)
		// 	angle --;

		// sector = angle / 60;

		ret = tmc4671_read_velocity(tmc4671_desc, &meas_speed);
		if (ret)
			// goto remove_pwmin_gpio;
			goto remove_tmc4671;

		/* TBD if kept. */
		if (meas_speed < 0)
			meas_speed *= (-1);

		/* Error calculation. */
		error_speed = ref_speed - meas_speed;

		/* PI Implementation. */
		// error_speed = (QBL4208_RATED_SPEED / error_speed) * 1000000;

		// y1 = (error_speed * KP);
		// if (y1 > 1000000)
		// 	y1 = 1000000;
		// else if (y1 < 0)
		// 	y1 = 0;

		// duty_cycle = (40000 * y1) / 1000000;

		// ret = no_os_pwm_set_duty_cycle(pwm_sector_sel, duty_cycle);
		// if (ret)
		// 	goto remove_pwmin_gpio;

		// ret = no_os_gpio_get_value(pwmin_gpio, &pwm_state);
		// if (ret)
		// 	goto remove_pwmin_gpio;

		// if (pwm_state == NO_OS_GPIO_HIGH) {
		// 	ret = tmc6100_bldc_sel_sector(tmc6100_desc, sector,
		// 				      tmc6100_sequence_1);
		// 	if (ret)
		// 		goto remove_pwmin_gpio;
		// } else {
		// 	ret = tmc6100_bldc_sel_sector(tmc6100_desc, sector,
		// 				      tmc6100_sequence_2);
		// 	if (ret)
		// 		goto remove_pwmin_gpio;
		// }

		/* UD Scalling. */
		error_speed *= KP;
		if (error_speed > 0)
			ud += 0x100;
		else
			ud -= 0x100;

		ret = tmc6100_reg_write(tmc6100_desc, TMC4671_UQ_UD_EXT_REG, ud);
		if (ret)
			goto remove_tmc4671;

		no_os_udelay(20);
	}

// remove_pwmin_gpio:
// 	no_os_gpio_remove(pwmin_gpio);
// remove_pwm_sector:
// 	no_os_pwm_remove(pwm_sector_sel);
remove_tmc4671:
	tmc4671_remove(tmc4671_desc);
remove_tmc6100:
	tmc6100_remove(tmc6100_desc);
exit:
	if (ret)
		pr_info("Error!\n");
	return ret;
}
