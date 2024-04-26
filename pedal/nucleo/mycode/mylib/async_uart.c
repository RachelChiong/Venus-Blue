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

#include "async_uart.h"
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(UART, LOG_LEVEL_INF);

// semaphore to indicate whether tx is being used or is finished (ready)
K_SEM_DEFINE(tx_finished, 1, 1);
// semaphore to indicate whether rx is disabled
K_SEM_DEFINE(rx_disabled, 0, 1);
K_SEM_DEFINE(tx_lock, 1, 1);

// json packets can be pretty large
#define UART_TX_PIPE_SIZE 4096
#define UART_RX_PIPE_SIZE 4096
K_PIPE_DEFINE(tx_pipe, UART_TX_PIPE_SIZE, 4);
K_PIPE_DEFINE(rx_pipe, UART_RX_PIPE_SIZE, 4);   // read uart recieve byte stream

#define UART_DEVICE_NODE DT_NODELABEL(uart0)
#define UART_BUF_SIZE 32

/* microseconds */
#define UART_TX_TIMEOUT_US 1000
#define UART_RX_TIMEOUT_US 1000

/* milliseconds */
#define UART_TX_RETRY_MS 3

// Async UART primary buffers
uint8_t uart_tx_buf[UART_BUF_SIZE];
uint8_t uart_double_buffer[2][UART_BUF_SIZE];
uint8_t *uart_buf_next = uart_double_buffer[1];

static const struct device *dev_uart = DEVICE_DT_GET(UART_DEVICE_NODE);

/*
 * uart_tx_get_from_pipe()
 * -----------------------
 * Transmit some bytes from internal TX pipe.
 *
 * Returns
 * - Number of bytes written
 */
static int uart_tx_get_from_pipe(void)
{
    size_t bytes_read;

    // Fetch bytes from TX pipe
    k_pipe_get(&tx_pipe, uart_tx_buf, sizeof(uart_tx_buf), &bytes_read, 0,
               K_NO_WAIT);

    if (bytes_read > 0) {
        // Send bytes over UART
        uart_tx(dev_uart, uart_tx_buf, bytes_read, UART_TX_TIMEOUT_US);
    }
    return bytes_read;
}

/*
 * async_uart_callback()
 * ---------------------
 * Callback to service asynchronous uart events.
 */
void async_uart_callback(const struct device *uart_dev, struct uart_event *evt,
                         void *user_data)
{
    switch (evt->type) {
    case UART_TX_DONE:
        // If there is more data in the TX pipe, transmit some more
        if (uart_tx_get_from_pipe() == 0) {
            // Or release the semaphore to indicate TX is not transmitting.
            k_sem_give(&tx_finished);
        }
        LOG_DBG("UART_TX_DONE");
        break;

    case UART_RX_RDY:
        int ret;
        size_t bytes_written;
        // write received bytes to rx buffer
        ret = k_pipe_put(&rx_pipe, evt->data.rx.buf + evt->data.rx.offset,
                         evt->data.rx.len, &bytes_written, evt->data.rx.len,
                         K_NO_WAIT);
        if (ret) {
            LOG_ERR("Uart RX pipe full!");
        }
        LOG_DBG("UART_RX_RDY");
        break;

    case UART_RX_BUF_REQUEST:
        // Provide async API with buffer to use for RX bytes
        uart_rx_buf_rsp(dev_uart, uart_buf_next, UART_BUF_SIZE);
        LOG_DBG("UART_RX_BUF_REQUEST");
        break;

    case UART_RX_BUF_RELEASED:
        // Buffer no longer used, use it again for next buffer request
        uart_buf_next = evt->data.rx_buf.buf;
        LOG_DBG("UART_RX_BUF_RELEASED");
        break;

    case UART_RX_DISABLED:
        LOG_WRN("UART_RX_DISABLED");
        // re-enable rx by resetting uart.
        async_uart_init();
        break;

    case UART_RX_STOPPED:
        LOG_WRN("UART_RX_STOPPED %i", evt->data.rx_stop.reason);
        break;

    case UART_TX_ABORTED:
        LOG_WRN("UART_TX_ABORTED");
        break;

    default:
        LOG_WRN("UART_OTHER_EVENT");
        break;
    }
}

// Configuration settings for uart
struct uart_config uart_cfg = {
    .baudrate = 115200,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    .data_bits = UART_CFG_DATA_BITS_8,
};

void async_uart_init(void)
{
    if (dev_uart == NULL) {
        LOG_ERR("Failed to get UART binding\n");
        return;
    }

    uart_configure(dev_uart, &uart_cfg);
    uart_callback_set(dev_uart, async_uart_callback, NULL);
    uart_rx_enable(dev_uart, uart_double_buffer[0], UART_BUF_SIZE,
                   UART_RX_TIMEOUT_US);
}

size_t async_uart_send(const uint8_t *data_ptr, size_t data_len)
{
    k_sem_take(&tx_lock, K_FOREVER);
    // Try to move the data into the TX pipe
    size_t bytes_written;

    k_pipe_put(&tx_pipe, data_ptr, data_len, &bytes_written, 0, K_NO_WAIT);

    // If UART TX is not already transmitting, start it
    if (k_sem_take(&tx_finished, K_NO_WAIT) == 0) {
        uart_tx_get_from_pipe();
    }
    k_sem_give(&tx_lock);

    return bytes_written;
}