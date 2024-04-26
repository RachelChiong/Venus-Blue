#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "async_uart.h"
#include "pc_json_requests.h"

LOG_MODULE_REGISTER(PC_JSON_REQUESTS, LOG_LEVEL_DBG);

#define JSON_REQUESTS_THREAD_STACKSIZE 8192
#define JSON_REQUESTS_THREAD_PRIORITY  11
#define MAX_REQUEST_SIZE 2048

char *strdup(const char *string) {
    char *res = k_malloc(strlen(string) + 1);
    strcpy(res, string);
    return res;
}

int json_encode_send_via_uart(const char *bytes, size_t len, void *data) {
    return async_uart_send(bytes, len) == len ? 0 : -1;
}

void send_pedal_json(struct pedal_data data) {
    char buf[256];
    json_obj_encode_buf(pedal_data_descr, ARRAY_SIZE(pedal_data_descr), &data, buf, 256);
    async_uart_send(buf, strlen(buf));
    async_uart_send("\n", 1);
}

void handle_request(char *json_request) {
    // LOG_DBG("Got Request: %s", json_request);
    // struct pc_request request;
    // json_obj_parse(json_request, strlen(json_request), pc_request_descr, ARRAY_SIZE(pc_request_descr), &request);
    LOG_ERR("Unknown request from PC");
}

/*
 * json_requests_thread()
 * ----------------------
 * Zephyr thread to continually chunk UART rx byte stream into JSON request
 * packets.
 *
 * Byte stream is read from rx_pipe.
 */
void pc_json_requests_thread(const void *, const void *, const void *)
{
    uint8_t buf[MAX_REQUEST_SIZE], *ptr = buf;
    size_t bytes_read;
    // Json objects are sent one line at a time.
    while (1) {
        k_pipe_get(&rx_pipe, ptr, 1, &bytes_read, 1, K_FOREVER);
        if (*ptr == '\n') {
            *ptr = 0;
            ptr = buf;
            handle_request(buf);
        } else {
            ptr += bytes_read;
        }
    }
}

K_THREAD_DEFINE(json_requests_thread, JSON_REQUESTS_THREAD_STACKSIZE, pc_json_requests_thread,
                NULL, NULL, NULL, JSON_REQUESTS_THREAD_PRIORITY, 0, 0);