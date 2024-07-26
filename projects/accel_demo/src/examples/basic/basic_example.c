/***************************************************************************//**
 *   @file   basic_example.c
 *   @brief  BASIC example source for accel project
 *   @author Radu Sabau (radu.sabau@analog.com)
********************************************************************************
 * Copyright 2024(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "stdio.h"
#include "basic_example.h"
#include "common_data.h"
#include "adxl355.h"
#include "ssd_1306.h"
#include "display.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_units.h"

static int flip(int val, int num_dec)
{
	int aux = 0, i = 0;

	while (i < num_dec) {
		aux = (aux * 10) + val % 10;
		val /= 10;
		i++;
	}

	return aux;
}

void get_msg(int integger, int frac, char *msg, char chan)
{
	int i = 3, max = 9;

	if (frac < 0) {
		integger *= -1;
		frac *= -1;
		msg[2] = '-';
		max = 11;
	} else {
		msg[2] = '+';
	}

	frac = flip(frac, 9);

	msg[0] = chan;
	msg[1] = ':';
	if (integger > 10) {
		msg[i + 1] = integger % 10 + 0x30;
		msg[i] = integger / 10 + 0x30;
		i += 2;
	} else {
		msg[i] = integger % 10 + 0x30;
		i++;
	}

	msg[i] = '.';
	i++;

	while (frac && i < max) {
		msg[i] = (frac % 10) + 0x30;
		frac /= 10;
		i++;
	}
}

/**
 * @brief Dummy example main execution.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously the while(1) loop and will not return.
 */
int basic_example_main()
{
	struct no_os_gpio_desc *vbat;
	struct no_os_gpio_desc *vdd;
	struct adxl355_dev *adxl355_desc;
	struct display_dev *display;
	char msg[] = "acceleration :", xmsg[12], ymsg[12], zmsg[12];
	int ret;

	// Display
	ret = no_os_gpio_get(&vbat, &vbat_pin);
	if (ret)
		return ret;

	ret = no_os_gpio_direction_output(vbat, NO_OS_GPIO_HIGH);
	if (ret)
		goto remove_vbat;

	ret = no_os_gpio_get(&vdd, &vdd_pin);
	if (ret)
		goto remove_vbat;

	ret = no_os_gpio_direction_output(vdd, NO_OS_GPIO_HIGH);
	if (ret)
		goto remove_vdd;

	ret = display_init(&display, &display_ip);
	if (ret)
		goto remove_vdd;

	ret = display_on(display);
	if (ret)
		goto remove_display;

	ret = display_clear(display);
	if (ret)
		goto remove_display;

	// ADXL355
	ret = adxl355_init(&adxl355_desc, adxl355_ip);
	if (ret)
		goto remove_adxl355;
	ret = adxl355_soft_reset(adxl355_desc);
	if (ret)
		goto remove_adxl355;
	ret = adxl355_set_odr_lpf(adxl355_desc, ADXL355_ODR_3_906HZ);
	if (ret)
		goto remove_adxl355;
	ret = adxl355_set_op_mode(adxl355_desc, ADXL355_MEAS_TEMP_ON_DRDY_OFF);
	if (ret)
		goto remove_adxl355;

	struct adxl355_frac_repr x;
	struct adxl355_frac_repr y;
	struct adxl355_frac_repr z;

	while(1) {
		ret = adxl355_get_xyz(adxl355_desc,&x, &y, &z);
		if (ret)
			goto remove_adxl355;

		pr_info(" x=%d"".%09u", (int)x.integer, (abs)(x.fractional));
		pr_info(" y=%d"".%09u", (int)y.integer, (abs)(y.fractional));
		pr_info(" z=%d"".%09u \n", (int)z.integer,(abs)(z.fractional));

		get_msg((int)x.integer, x.fractional, xmsg, 'x');
		get_msg((int)y.integer, y.fractional, ymsg, 'y');
		get_msg((int)z.integer, z.fractional, zmsg, 'z');

		ret = display_print_string(display, msg, 0, 0);
		if (ret)
			goto remove_adxl355;

		ret = display_print_string(display, xmsg, 1, 0);
		if (ret)
			goto remove_adxl355;

		ret = display_print_string(display, ymsg, 2, 0);
		if (ret)
			goto remove_adxl355;

		ret = display_print_string(display, zmsg, 3, 0);
		if (ret)
			goto remove_adxl355;

		no_os_mdelay(500);
		ret = display_clear(display);
		if (ret)
			goto remove_adxl355;
	}

remove_adxl355:
	adxl355_remove(adxl355_desc);
remove_display:
	display_remove(display);
remove_vdd:
	no_os_gpio_remove(vdd);
remove_vbat:
	no_os_gpio_remove(vbat);

	return ret;
}
