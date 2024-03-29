#ifndef __AD7920_H__
#define __AD7920_H__

#include <stdbool.h>
#include "no_os_spi.h"
#include "no_os_gpio.h"

struct ad7920_desc {
	struct no_os_spi_desc *comm_desc;
};

struct ad7920_init_param {
	struct no_os_spi_init_param *comm_param;
};

int ad7920_get_raw(struct ad7920_desc *desc);

int ad7920_init(struct ad7920_desc **desc,
		struct ad7920_init_param *init_param);

int ad7920_remove(struct ad7920_desc *desc);

#endif /* __AD7920_H__ */
