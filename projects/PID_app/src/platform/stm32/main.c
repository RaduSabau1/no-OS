#include "platform_includes.h"
#include "common_data.h"
#include "no_os_error.h"
#include "main_example.h"

int main()
{

	int ret = -EINVAL;

	struct no_os_uart_desc *uart_desc;

	stm32_init();

	ret = no_os_uart_init(&uart_desc, &pid_app_uart_ip);
	if (ret)
		return ret;

	no_os_uart_stdio(uart_desc);

	ret = main_example_main();

	no_os_uart_remove(uart_desc);

	return ret;
}
