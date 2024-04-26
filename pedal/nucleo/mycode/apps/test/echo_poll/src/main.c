/*
 * Copyright (c) 2022 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <string.h>

/* change this to any other UART peripheral if desired */
#define UART_DEVICE_NODE DT_NODELABEL(uart1)


static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

/*
 * Print a null-terminated string character by character to the UART interface
 */
void print_uart(char *buf)
{
	int msg_len = strlen(buf);

	for (int i = 0; i < msg_len; i++) {
		uart_poll_out(uart_dev, buf[i]);
	}
}

int main(void)
{

	if (!device_is_ready(uart_dev)) {
		printk("UART device not found!");
		return 0;
	}

	printk("Hello! I'm your echo bot.\r\n");
	print_uart("Tell me something and press enter:\r\n");

	/* indefinitely wait for input from the user */
	uint8_t c;
	int ret;


	while (1) {
		if (uart_poll_in(uart_dev, &c) == 0) {
			printk("Echo: ");
			printk("%c", c);
			printk("\r\n");
		}
		k_msleep(1);
	}
	return 0;
}
