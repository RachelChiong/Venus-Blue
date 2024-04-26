/**
 **************************************************************
 * @file include/pc_json_requests.h
 * @author James King - 47443732
 * @date 26/04/2024
 * @brief Library that supports json communication to the host
 * pi.
 ***************************************************************
 */

#ifndef __PC_JSON_REQUESTS_H__
#define __PC_JSON_REQUESTS_H__

#include <zephyr/data/json.h>

struct pedal_data {
     int32_t x;
     int32_t y;
};

static const struct json_obj_descr pedal_data_descr[] = {
    JSON_OBJ_DESCR_PRIM(struct pedal_data, x, JSON_TOK_NUMBER),
    JSON_OBJ_DESCR_PRIM(struct pedal_data, y, JSON_TOK_NUMBER),
};

void send_pedal_json(struct pedal_data data);

#endif