#ifndef _TMC4671_H_
#define _TMC4671_H_

#include <stdint.h>
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_uart.h"
#include "no_os_util.h"

#define TMC4671_CHIPINFO_DATA_REG			0x00
#define TMC4671_CHIPINFO_ADDR_REG			0x01
#define TMC4671_ADC_RAW_DATA_REG			0x02
#define TMC4671_ADC_RAW_ADDR_REG			0x03
#define TMC4671_DS_ADC_MCFG_B_MCFG_A_REG		0x04
#define TMC4671_DS_ADC_MCLK_A_REG			0x05
#define TMC4671_DS_ADC_MCLK_B_REG			0x06
#define TMC4671_DS_ADC_MDEC_B_MDEC_A_REG		0x07
#define TMC4671_ADC_I1_SCALE_OFFSET_REG			0x08
#define TMC4671_ADC_I0_SCALE_OFFSET_REG			0x09
#define TMC4671_ADC_I_SELECT_REG			0x0A
#define TMC4671_ADC_I1_I0_EXT_REG			0x0B
#define TMC4671_DS_ANALOG_INPUT_STAGE_CFG_REG		0x0C
#define TMC4671_AENC_0_SCALE_OFFSET_REG			0x0D
#define TMC4671_AENC_1_SCALE_OFFSET_REG			0x0E
#define TMC4671_AENC_2_SCALE_OFFSET_REG			0x0F
#define TMC4671_AENC_SELECT_REG				0x11
#define TMC4671_ADC_IWY_IUX_REG				0x12
#define TMC4671_ADC_IV_REG				0x13
#define TMC4671_AENC_WY_UX_REG				0x15
#define TMC4671_AENC_VN_REG				0x16
#define TMC4671_PWM_POLARITIES_REG			0x17
#define TMC4671_PWM_MAXCNT_REG				0x18
#define TMC4671_PWM_BBM_H_BBM_L_REG			0x19
#define TMC4671_PWM_SV_CHOP_REG				0x1A
#define TMC4671_MOTOR_TYPE_N_POLE_PAIRS_REG		0x1B
#define TMC4671_PHI_E_EXT_REG				0x1C
#define TMC4671_PHI_M_EXT_REG				0x1D
#define TMC4671_POSITION_EXT_REG			0x1E
#define TMC4671_OPENLOOP_MODE_REG			0x1F
#define TMC4671_OPENLOOP_ACCELERATION_REG		0x20
#define TMC4671_OPENLOOP_VELOCITY_TARGET_REG		0x21
#define TMC4671_OPENLOOP_VELOCITY_ACTUAL_REG		0x22
#define TMC4671_OPENLOOP_PHI_REG			0x23
#define TMC4671_UQ_UD_EXT_REG				0x24
#define TMC4671_ABN_DECODER_MODE_REG			0x25
#define TMC4671_ABN_DECODER_PPR_REG			0x26
#define TMC4671_ABN_DECODER_COUNT_REG			0x27
#define TMC4671_ABN_DECODER_COUNT_N_REG			0x28
#define TMC4671_ABN_DECODER_PHI_E_PHI_M_OFFSET_REG	0x29
#define TMC4671_ABN_DECODER_PHI_E_PHI_M_REG		0x2A
#define TMC4671_ABN_2_DECODER_MODE_REG			0x2C
#define TMC4671_ABN_2_DECODER_PPR_REG			0x2D
#define TMC4671_ABN_2_DECODER_COUNT_REG			0x2E
#define TMC4671_ABN_2_DECODER_COUNT_N_REG		0x2F
#define TMC4671_ABN_2_DECODER_PHI_M_OFFSET_REG		0x30
#define TMC4671_ABN_2_DECODER_PHI_M_REG			0x31
#define TMC4671_HALL_MODE_REG				0x33
#define TMC4671_HALL_POSITION_060_000_REG		0x34
#define TMC4671_HALL_POSITION_180_120_REG		0x35
#define TMC4671_HALL_POSITION_300_240_REG		0x36
#define TMC4671_HALL_PHI_E_PHI_M_OFFSET_REG		0x37
#define TMC4671_HALL_DPHI_MAX_REG			0x38
#define TMC4671_HALL_PHI_E_INTERPOLATED_PHI_E_REG	0x39
#define TMC4671_HALL_PHI_M_REG				0x3A
#define TMC4671_AENC_DECODER_MODE_REG			0x3B
#define TMC4671_AENC_DECODER_N_THRESHOLD_REG		0x3C
#define TMC4671_AENC_DECODER_PHI_A_RAW_REG		0x3D
#define TMC4671_AENC_DECODER_PHI_A_OFFSET_REG		0x3E
#define TMC4671_AENC_DECODER_PHI_A_REG			0x3F
#define TMC4671_AENC_DECODER_PPR_REG			0x40
#define TMC4671_AENC_DECODER_COUNT_REG			0x41
#define TMC4671_AENC_DECODER_COUNT_N_REG		0x42
#define TMC4671_AENC_DECODER_PHI_E_PHI_M_OFFSET_REG	0x45
#define TMC4671_AENC_DECODER_PHI_E_PHI_M_REG		0x46
#define TMC4671_AENC_DECODER_POSITION_REG		0x47
#define TMC4671_CONFIG_DATA_REG				0x4D
#define TMC4671_CONFIG_ADDR_REG				0x4E
#define TMC4671_VELOCITY_SELECTION_REG			0x50
#define TMC4671_POSITION_SELECTION_REG			0x51
#define TMC4671_PHI_E_SELECTION_REG			0x52
#define TMC4671_PHI_E_REG				0x53
#define TMC4671_PID_FLUX_P_FLUX_I_REG			0x54
#define TMC4671_PID_TORQUE_P_TORQUE_I_REG		0x56
#define TMC4671_PID_VELOCITY_P_VELOCITY_I_REG		0x58
#define TMC4671_PID_POSITION_P_POSITION_I_REG		0x5A
#define TMC4671_PID_TORQUE_FLUX_TARGET_DDT_LIMITS_REG	0x5C
#define TMC4671_PIDOUT_UQ_UD_LIMITS_REG			0x5D
#define TMC4671_PID_TORQUE_FLUX_LIMITS_REG		0x5E
#define TMC4671_PID_ACCELERATION_LIMIT_REG		0x5F
#define TMC4671_PID_VELOCITY_LIMIT_REG			0x60
#define TMC4671_PID_POSITION_LIMIT_LOW_REG		0x61
#define TMC4671_PID_POSITION_LIMIT_HIGH_REG		0x61
#define TMC4671_MODE_RAMP_MODE_MOTION_REG		0x63
#define TMC4671_PID_TORQUE_FLUX_TARGET_REG		0x64
#define TMC4671_PID_TORQUE_FLUX_OFFSET_REG		0x65
#define TMC4671_PID_VELOCITY_TARGET_REG			0x66
#define TMC4671_PID_VELOCITY_OFFSET_REG			0x67
#define TMC4671_PID_POSITION_TARGET_REG			0x68
#define TMC4671_PID_TORQUE_FLUX_ACTUAL_REG		0x69
#define TMC4671_PID_VELOCITY_ACTUAL_REG			0x6A
#define TMC4671_PID_POSITION_ACTUAL_REG			0x6B
#define TMC4671_PID_ERROR_DATA_REG			0x6C
#define TMC4671_PID_ERROR_ADDR_REG			0x6D
#define TMC4671_INTERIM_DATA_REG			0x6E

#define TMC4671_RW_MASK					NO_OS_BIT(7)
#define TMC4671_VELOCITY_SELECTION_MASK			NO_OS_GENMASK(7, 0)
#define TMC4671_POSITION_SELECTION_MASK			NO_OS_GENMASK(7, 0)
#define TMC4671_PHI_E_SELECTION_MASK			NO_OS_GENMASK(7, 0)
#define TMC4671_PHI_E_MASK				NO_OS_GENMASK(15, 0)
#define TMC4671_HALL_MODE_MASK(x)			NO_OS_BIT(x)
#define TMC4671_MOTOR_TYPE_MASK				NO_OS_GENMASK(23, 16)
#define TMC4671_PWM_MAXCNT_MASK				NO_OS_GENMASK(11, 0)
#define TMC4671_PWM_SV_CHOP_MASK			NO_OS_GENMASK(7, 0)
#define TMC4671_MODE_MOTION_MASK			NO_OS_GENMASK(7, 0)

enum tmc4671_motor_type {
	TMC4671_NO_MOTOR,
	TMC4671_SINGLE_PHASE_DC,
	TMC4671_TWO_PHASE_STEPPER,
	TMC4671_THREE_PHASE_BLDC,
};

enum tmc4671_source_sel {
	TMC4671_PHI_E_SELECTION,
	TMC4671_PHI_E_EXT,
	TMC4671_PHI_E_OPENLOOP,
	TMC4671_PHI_E_ABN,
	TMC4671_PHI_E_HAL = 5,
	TMC4671_PHI_E_AENC,
	TMC4671_PHI_A_AENC,
	TMC4671_PHI_M_ABN = 9,
	TMC4671_PHI_M_ABN_2,
	TMC4671_PHI_M_AENC,
	TMC4671_PHI_M_HAL,
};

enum tmc4671_hall_mode_sel {
	TMC4671_HALL_POLARITY = 0,
	TMC4671_HALL_SYNC_PWM = 4,
	TMC4671_HALL_INTERPOLATION = 8,
	TMC4671_HALL_DIRECTION = 12,
};

enum tmc4671_hall_mode_motion_sel {
	TMC4671_STOPPED_MODE,
	TMC4671_TORQUE_MODE,
	TMC4671_VELOCITY_MODE,
	TMC4671_POSITION_MODE,
	TMC4671_PRBS_FLUX_MODE,
	TMC4671_PRBS_TORQUE_MODE,
	TMC4671_PRBS_VELOCITY_MODE,
	TMC4671_PRBS_POSITION_MODE,
	TMC4671_UQ_UD_EXT,
	TMC4671_ENC_INIT_MOVE,
	TMC4671_AGPI_A_TORQUE_MODE,
	TMC4671_AGPI_A_VELOCITY_MODE,
	TMC4671_AGPI_A_POSITION_MODE,
	TMC4671_PWM_I_TORQUE_MODE,
	TMC4671_PWM_I_VELOCITY_MODE,
	TMC4671_PWM_I_POSITION_MODE,
};

enum tmc4671_pwm_sv_chop_sel {
	TMC4671_FREE_RUNNING,
	TMC4671_LS_ON,
	TMC4671_HS_ON,
	TMC4671_LS_CHOPPER = 5,
	TMC4671_HS_CHOPPER = 6,
	TMC4671_PWM_CENTER,
	TMC4671_PWM_OFF = 256,
};

struct tmc4671_init_param {
	struct no_os_spi_init_param *spi_param;
	struct no_os_uart_init_param *uart_param;
	struct no_os_gpio_init_param *rstn_param;

	enum tmc4671_motor_type motor_type;
	uint8_t no_pole_pairs;
};

struct tmc4671_desc {
	struct no_os_spi_desc *spi_desc;
	struct no_os_uart_desc *uart_desc;
	struct no_os_gpio_desc *rstn_desc;

	enum tmc4671_motor_type motor_type;
	uint8_t no_pole_pairs;
	uint8_t buff[6];
};

int tmc4671_reg_read(struct tmc4671_desc *desc, uint8_t reg, uint32_t *val);

int tmc4671_reg_write(struct tmc4671_desc *desc, uint8_t reg, uint32_t val);

int tmc4671_reg_update(struct tmc4671_desc *desc, uint8_t reg, uint32_t mask,
		       uint32_t val);

int tmc4671_velocity_sel(struct tmc4671_desc *desc,
			 enum tmc4671_source_sel sel);

int tmc4671_position_sel(struct tmc4671_desc *desc,
			 enum tmc4671_source_sel sel);

int tmc4671_phi_e_sel(struct tmc4671_desc *desc, enum tmc4671_source_sel sel);

int tmc4671_read_angle(struct tmc4671_desc *desc, uint32_t *angle);

int tmc4671_read_velocity(struct tmc4671_desc *desc, int32_t *velocity);

int tmc4671_set_hall_mode(struct tmc4671_desc *desc,
			  enum tmc4671_hall_mode_sel sel,
			  bool enable);

int tmc4671_get_hall_mode(struct tmc4671_desc *desc,
			  enum tmc4671_hall_mode_sel sel,
			  bool *enable);

int tmc4671_set_pwm_freq(struct tmc4671_desc *desc, uint32_t freq);

int tmc4671_set_pwm_bbm(struct tmc4671_desc *desc, uint8_t bbm);

int tmc4671_set_pwm_sv_chop(struct tmc4671_desc *desc,
			    enum tmc4671_pwm_sv_chop_sel sv_chop);

int tmc4671_set_mode_motion(struct tmc4671_desc *desc,
			    enum tmc4671_hall_mode_motion_sel mode_motion);

int tmc4671_init(struct tmc4671_desc **desc,
		 struct tmc4671_init_param *init_param);

int tmc4671_remove(struct tmc4671_desc *desc);

#endif /* _TMC4671_H_ */
