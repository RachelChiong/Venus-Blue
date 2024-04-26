#include "async_uart.h"
#include "pc_json_requests.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(PRAC3_MAIN, LOG_LEVEL_DBG);

int main()
{
    // Initialise devices
    async_uart_init();
    for (;;) {
        k_msleep(1000);
        struct pedal_data data = {2323, -1204};
        send_pedal_json(data);
    }
}