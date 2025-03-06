#include "platform_includes.h"
#include "common_data.h"
#include "no_os_error.h"

#ifdef EXT_LOOP_EXAMPLE
#include "ext_loop_example.h"
#endif

#ifdef INT_LOOP_EXAMPLE
#include "int_loop_example.h"
#endif

#ifdef OPEN_LOOP_EXAMPLE
#include "open_loop_example.h"
#endif

int main()
{
	int ret = -EINVAL;

	struct no_os_uart_desc *uart_desc;

	stm32_init();

	ret = no_os_uart_init(&uart_desc, &pid_app_uart_ip);
	if (ret)
		return ret;

	no_os_uart_stdio(uart_desc);

	ret = ext_loop_example_main();

	no_os_uart_remove(uart_desc);

	return ret;
}
