#include "platform_includes.h"
#include "common_data.h"
#include "no_os_error.h"

#ifdef EXT_LOOP_EXAMPLE
#include "ext_loop_example.h"
#endif

#ifdef EXT_TRANSF_EXAMPLE
#include "ext_transf_example.h"
#endif

#ifdef EXT_CTRL_EXAMPLE
#include "ext_ctrl_example.h"
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

// #ifdef EXT_LOOP_EXAMPLE
	ret = ext_loop_example_main();
// #endif

#ifdef EXT_TRANSF_EXAMPLE
	ret = ext_transf_example_main();
#endif

#ifdef EXT_CTRL_EXAMPLE
	ret = ext_ctrl_example_main();
#endif

	no_os_uart_remove(uart_desc);

#if (EXT_LOOP_EXAMPLE + EXT_TRANSF_EXAMPLE + EXT_CTRL_EXAMPLE > 1)
#error Selected example projects cannot be enabled at the same time. \
Please enable ony one example and rebuild thhe project.
#endif

	return ret;
}
