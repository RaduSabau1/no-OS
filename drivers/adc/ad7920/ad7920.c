#include <stdlib.h>
#include "no_os_error.h"
#include "no_os_alloc.h"
#include "no_os_error.h"

int ad7920_get_raw(struct ad7920_desc *desc)
{
	uint16_t raw_voltage;
	uint8_t buff[2];
	int ret;

	ret = no_os_spi_write_and_read(desc->comm_desc, buff, 2);
	if (ret)
		return ret;

	raw_voltage = no_os_get_unaligned_be16(buff);

	return (int)raw_voltage;
}

int ad7920_init(struct ad7920_desc **desc,
		struct ad7920_init_param *init_param)
{
	struct ad7920_desc *descriptor;
	int ret;

	descriptor = (struct ad7920_desc *)no_os_calloc(1, sizeof(*descriptor));
	if (ret)
		return -ENOMEM;

	ret = no_os_spi_init(&descriptor->comm_desc, init_param->comm_param);
	if (ret)
		goto error;

	*desc = descriptor;

	return 0;

error:
	no_os_free(descriptor);
	return ret;
}

int ad7920_remove(struct ad7920_desc *desc)
{
	no_os_spi_remove(desc->comm_desc);
	no_os_free(desc);
	return 0;
}
