#include <stdint.h>

#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

#define TMC6100_GCONF_REG		0x00
#define TMC6100_GSTAT_REG		0x01
#define TMC6100_IOIN_REG		0x04
#define TMC6100_OTP_PROG_REG		0x06
#define TMC6100_OTP_READ_REG		0x07
#define TMC6100_FACTORY_CONF_REG	0x08
#define TMC6100_SHORT_CONF_REG		0x09
#define TMC6100_DRV_CONF_REG		0x0A

#define TMC6100_RW_MASK			NO_OS_BIT(7)

#define TMC6100_BLDC_UH_WL_SEQ		0x21
#define TMC6100_BLDC_VH_WL_SEQ		0x09
#define TMC6100_BLDC_UL_VH_SEQ		0x18
#define TMC6100_BLDC_UL_WH_SEQ		0x12
#define TMC6100_BLDC_VL_WH_SEQ		0x06
#define TMC6100_BLDC_UH_VL_SEQ		0x24

enum tmc6100_out_pin_sel {
	TMC6100_UH,
	TMC6100_UL,
	TMC6100_VH,
	TMC6100_VL,
	TMC6100_WH,
	TMC6100_WL,
};

enum tmc6100_bldc_sector {
	TMC6100_BLDC_SECTOR_1,
	TMC6100_BLDC_SECTOR_2,
	TMC6100_BLDC_SECTOR_3,
	TMC6100_BLDC_SECTOR_4,
	TMC6100_BLDC_SECTOR_5,
	TMC6100_BLDC_SECTOR_6,
};

enum tmc6100_fault_sel {
	TMC6100_FAULT_OVC_OTP_ONLY,
	TMC6100_FAULT_ALL,
};

enum tmc6100_interface {
	TMC6100_L_H_INDIVIDUAL,
	TMC6100_H_CONTROL_L_ENABLE,
};

enum tmc6100_status_sel {
	TMC6100_RESET_STATUS,
	TMC6100_DRVOTPW_STATUS,
	TMC6100_DRVOT_STATUS,
	TMC6100_UVCP_STATUS,
	TMC6100_SHORTDET_U_STATUS,
	TMC6100_SHORT2GNDU_STATUS,
	TMC6100_SHORT2VSU_STATUS,
	TMC6100_SHORTDET_V_STATUS = 8,
	TMC6100_SHORT2GNDV_STATUS,
	TMC6100_SHORT2VSV_STATUS,
	TMC6100_SHORTDET_W_STATUS = 12,
	TMC6100_SHORT2GNDW_STATUS,
	TMC6100_SHORT2VSW_STATUS,
};

enum tmc6100_short_sens_sel {
	TMC6100_SHORT2VSUPPLY_SENS,
	TMC6100_SHORT2VGND_SENS,
};

enum tmc6100_short_filter_bw {
	TMC6100_SHORT_FILTER_BW_100NS,
	TMC6100_SHORT_FILTER_BW_1US,
	TMC6100_SHORT_FILTER_BW_2US,
	TMC6100_SHORT_FILTER_BW_3US,
};

struct tmc6100_init_param {
	struct no_os_spi_init_param *comm_param;

	struct no_os_gpio_init_param *drv_en_param;
	struct no_os_gpio_init_param *spe_param;

	struct no_os_gpio_init_param *uh_param;
	struct no_os_gpio_init_param *ul_param;
	struct no_os_gpio_init_param *vh_param;
	struct no_os_gpio_init_param *vl_param;
	struct no_os_gpio_init_param *wh_param;
	struct no_os_gpio_init_param *wl_param;
};

struct tmc6100_desc {
	struct no_os_spi_desc *comm_desc;

	struct no_os_gpio_desc *drv_en_desc;
	struct no_os_gpio_desc *spe_desc;

	struct no_os_gpio_desc *wvu_desc[6];
};

int tmc6100_reg_read(struct tmc6100_desc *desc, uint8_t reg, uint32_t *val);

int tmc6100_reg_write(struct tmc6100_desc *desc, uint8_t reg, uint32_t val);

int tmc6100_reg_update(struct tmc6100_desc *desc, uint8_t reg, uint32_t mask,
		       uint32_t val);

int tmc6100_bldc_create_seq(struct tmc6100_desc *desc,
			    enum tmc6100_out_pin_sel phase_start,
			    uint8_t *sequence);

int tmc6100_bldc_sel_sector(struct tmc6100_desc *desc,
			    enum tmc6100_bldc_sector sector,
			    uint8_t *sequence);

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
		 struct tmc6100_init_param *init_param);

int tmc6100_remove(struct tmc6100_desc *desc);
