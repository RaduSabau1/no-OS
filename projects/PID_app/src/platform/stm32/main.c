#include "platform_includes.h"
#include "common_data.h"
#include "no_os_error.h"

#ifdef EXT_OL_EXAMPLE
#include "ext_ol_example.h"
#endif

#ifdef INT_OL_THETA_EXAMPLE
#include "int_ol_theta_example.h"
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

#ifdef EXT_OL_EXAMPLE
	ret = ext_ol_example_main();
#endif

#ifdef INT_OL_THETA_EXAMPLE
	ret = int_ol_theta_example_main();
#endif

#ifdef INT_OL_DQWT_EXAMPLE
	ret = int_ol_dqwt_example_main();
#endif

#ifdef INT_OL_ALL_EXAMPLE
	ret = int_ol_all_example_main();
#endif

	no_os_uart_remove(uart_desc);

	return ret;
}
