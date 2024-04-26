/**
 **************************************************************
 * @file include/async_uart.h
 * @author James King - 47443732
 * @date 04/03/2024
 * @brief The Async UART Lib provides methods and pipes to
 *        read and transmit data over UART asynchronously.
 ***************************************************************
 * EXTERNAL FUNCTIONS
 ***************************************************************
 * async_uart_init() - Initialise async UART and enable rx
 * async_uart_send() - Send a byte buffer over uart
 ***************************************************************
 * EXTERNAL VARIABLES
 ***************************************************************
 * rx_pipe - Pipe that contains UART RX byte stream
 ***************************************************************
 */

#ifndef S4744373_ASYNC_UART_H
#define S4744373_ASYNC_UART_H
#include <zephyr/kernel.h>

// pipe that contains UART RX byte stream
extern struct k_pipe rx_pipe;

/*
 * async_uart_init()
 * -----------------
 * Initialise asynchronous uart on uart1 device. Sets appropriate callbacks and
 * enables rx.
 */
void async_uart_init(void);

/*
 * async_uart_send()
 * -----------------
 * Transmit a number of bytes via uart. Will immediately return after copying
 * as many bytes as possible into internal buffer.
 *
 * Bytes are then transmitted asynchronously from the internal buffer when TX
 * is available.
 *
 * Returns
 * - Number of bytes placed in internal pipe
 */
size_t async_uart_send(const uint8_t *data_ptr, size_t data_len);

#endif