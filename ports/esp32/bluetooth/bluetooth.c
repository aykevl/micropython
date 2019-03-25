/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ayke van Laethem
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#if MICROPY_PY_BLUETOOTH

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

#include "py/mperrno.h"
#include "py/runtime.h"
#include "extmod/modbluetooth.h"
#include <sys/queue.h>

typedef struct characteristic_node {
    mp_bt_characteristic_t *characteristic;
    LIST_ENTRY(characteristic_node) entries;
    LIST_HEAD(characteristic_node_descriptors, characteristic_node) descriptors;
} characteristic_node_t;

typedef struct profile_node {
    mp_bt_service_t *service;
    characteristic_node_t *prepared;
    LIST_ENTRY(profile_node) entries;
    LIST_HEAD(characteristic_node_list, characteristic_node) characteristics;
} profile_node_t;

LIST_HEAD(profile_node_list, profile_node) profiles;

// Semaphore to serialize asynchronous calls.
STATIC SemaphoreHandle_t mp_bt_call_complete;
STATIC esp_bt_status_t mp_bt_call_status;
STATIC union {
    // Ugly hack to return values from an event handler back to a caller.
    esp_gatt_if_t gatts_if;
    uint16_t      service_handle;
    uint16_t      attr_handle;
} mp_bt_call_result;

STATIC mp_bt_adv_type_t bluetooth_adv_type;
STATIC uint16_t bluetooth_adv_interval;
STATIC uint16_t bluetooth_app_id = 0; // provide unique number for each application profile

STATIC void mp_bt_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
STATIC void mp_bt_gatts_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

STATIC uint16_t global_mtu_size = 23;
STATIC uint16_t global_conn_id = 0xffff;
STATIC esp_gatt_if_t global_gatts_if = 0xff;
STATIC bool global_is_connected = false;

// Convert an esp_err_t into an errno number.
STATIC int mp_bt_esp_errno(esp_err_t err) {
    switch (err) {
    case 0:
        return 0;
    case ESP_ERR_NO_MEM:
        return MP_ENOMEM;
    case ESP_ERR_INVALID_ARG:
        return MP_EINVAL;
    default:
        return MP_EPERM; // fallback
    }
}

// Convert the result of an asynchronous call to an errno value.
STATIC int mp_bt_status_errno(void) {
    switch (mp_bt_call_status) {
    case ESP_BT_STATUS_SUCCESS:
        return 0;
    case ESP_BT_STATUS_NOMEM:
        return MP_ENOMEM;
    case ESP_BT_STATUS_PARM_INVALID:
        return MP_EINVAL;
    default:
        return MP_EPERM; // fallback
    }
}

// Initialize at early boot.
void mp_bt_init(void) {
    LIST_INIT(&profiles);
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    mp_bt_call_complete = xSemaphoreCreateBinary();
}

int mp_bt_enable(void) {
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t err = esp_bt_controller_init(&bt_cfg);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_bluedroid_init();
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_bluedroid_enable();
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_ble_gap_register_callback(mp_bt_gap_callback);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    err = esp_ble_gatts_register_callback(mp_bt_gatts_callback);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    return 0;
}

void mp_bt_disable(void) {
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
}

bool mp_bt_is_enabled(void) {
    return esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED;
}

STATIC esp_err_t mp_bt_advertise_start_internal(void) {
    esp_ble_adv_params_t ble_adv_params = {0,
        .adv_int_min       = bluetooth_adv_interval,
        .adv_int_max       = bluetooth_adv_interval,
        .adv_type          = bluetooth_adv_type,
        .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
        .channel_map       = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };
    return esp_ble_gap_start_advertising(&ble_adv_params);
}

int mp_bt_advertise_start(mp_bt_adv_type_t type, uint16_t interval, const uint8_t *adv_data, size_t adv_data_len, const uint8_t *sr_data, size_t sr_data_len) {
    if (adv_data != NULL) {
        esp_err_t err = esp_ble_gap_config_adv_data_raw((uint8_t*)adv_data, adv_data_len);
        if (err != 0) {
            return mp_bt_esp_errno(err);
        }
        // Wait for ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT
        xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
        if (mp_bt_call_status != 0) {
            return mp_bt_status_errno();
        }
    }

    if (sr_data != NULL) {
        esp_err_t err = esp_ble_gap_config_scan_rsp_data_raw((uint8_t*)sr_data, sr_data_len);
        if (err != 0) {
            return mp_bt_esp_errno(err);
        }
        // Wait for ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT
        xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
        if (mp_bt_call_status != 0) {
            return mp_bt_status_errno();
        }
    }

    bluetooth_adv_type = type;
    bluetooth_adv_interval = interval;
    esp_err_t err = mp_bt_advertise_start_internal();
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    // Wait for ESP_GAP_BLE_ADV_START_COMPLETE_EVT
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    return mp_bt_status_errno();
}

void mp_bt_advertise_stop(void) {
    esp_err_t err = esp_ble_gap_stop_advertising();
    if (err != 0) {
        return;
    }
    // Wait for ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
}

void _mp_bt_remove_profile(profile_node_t *profile) {
    LIST_REMOVE(profile, entries);
    free(profile);
}

characteristic_node_t *_mp_bt_find_char_node(profile_node_t *profile, mp_bt_characteristic_handle_t handle) {
    characteristic_node_t *chr_node = NULL;
    characteristic_node_t *descr_node = NULL;
    LIST_FOREACH(chr_node, &profile->characteristics, entries)
        if (chr_node->characteristic->value_handle == handle)
            return chr_node;
        else
            LIST_FOREACH(descr_node, &chr_node->descriptors, entries)
                if (descr_node->characteristic->value_handle == handle)
                    return descr_node;
    return chr_node;
}

int mp_bt_add_service(mp_bt_service_t *service, size_t num_characteristics, mp_bt_characteristic_t **characteristics) {
    // In ESP-IDF, a service is more than just a service, it's an
    // "application profile". One application profile contains exactly one
    // service. For details, see:
    // https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md

    // Register an application profile.
    profile_node_t *profile = malloc(sizeof(profile_node_t));
    profile->service = service;
    profile->prepared = NULL;
    LIST_INSERT_HEAD(&profiles, profile, entries);

    service->id = bluetooth_app_id;
    esp_err_t err = esp_ble_gatts_app_register(bluetooth_app_id);
    if (err != 0) {
        ESP_LOGE("bluetooth", "register app failed, error code =%x", err);
        _mp_bt_remove_profile(profile);
        return mp_bt_esp_errno(err);
    }
    bluetooth_app_id++;
    // Wait for ESP_GATTS_REG_EVT
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    if (mp_bt_call_status != 0) {
        _mp_bt_remove_profile(profile);
        return mp_bt_status_errno();
    }

    // Calculate the number of required handles.
    // This formula is a guess. I can't seem to find any documentation for
    // the required number of handles.
    uint16_t num_handle = 1 + num_characteristics * 2;
    for (size_t i = 0; i < num_characteristics; i++) {
        mp_bt_characteristic_t *characteristic = characteristics[i];
        mp_obj_list_t *descriptors = (mp_obj_list_t *)characteristic->descriptors;
        num_handle += descriptors->len * 2;
    }

    // Create the service.
    esp_gatt_srvc_id_t bluetooth_service_id;
    bluetooth_service_id.is_primary = true;
    bluetooth_service_id.id.inst_id = 0;
    bluetooth_service_id.id.uuid = service->uuid;

    err = esp_ble_gatts_create_service(service->gatts_if, &bluetooth_service_id, num_handle);
    if (err != 0) {
        ESP_LOGE("bluetooth", "create service failed, error code =%x", err);
        _mp_bt_remove_profile(profile);
        return mp_bt_esp_errno(err);
    }
    // Wait for ESP_GATTS_CREATE_EVT
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    if (mp_bt_call_status != 0) {
        _mp_bt_remove_profile(profile);
        return mp_bt_status_errno();
    }

    // Start the service.
    err = esp_ble_gatts_start_service(service->handle);
    if (err != 0) {
        ESP_LOGE("bluetooth", "start service failed, error code =%x", err);
        _mp_bt_remove_profile(profile);
        return mp_bt_esp_errno(err);
    }
    // Wait for ESP_GATTS_START_EVT
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    if (mp_bt_call_status != 0) {
        _mp_bt_remove_profile(profile);
        return mp_bt_status_errno();
    }

    LIST_INIT(&profile->characteristics);
    // Add each characteristic.
    for (size_t i = 0; i < num_characteristics; i++) {
        mp_bt_characteristic_t *characteristic = characteristics[i];

        esp_gatt_perm_t perm = 0;
        perm |= (characteristic->flags & MP_BLE_FLAG_READ) ? ESP_GATT_PERM_READ : 0;
        perm |= (characteristic->flags & MP_BLE_FLAG_WRITE) ? ESP_GATT_PERM_WRITE : 0;

        esp_gatt_char_prop_t property = 0;
        property |= (characteristic->flags & MP_BLE_FLAG_READ) ? ESP_GATT_CHAR_PROP_BIT_READ : 0;
        property |= (characteristic->flags & MP_BLE_FLAG_WRITE) ? ESP_GATT_CHAR_PROP_BIT_WRITE : 0;
        property |= (characteristic->flags & MP_BLE_FLAG_NOTIFY) ? ESP_GATT_CHAR_PROP_BIT_NOTIFY : 0;

        esp_attr_value_t char_val = {0};
        char_val.attr_max_len = MP_BT_MAX_ATTR_SIZE;
        char_val.attr_len = 0;
        char_val.attr_value = NULL;

        esp_attr_control_t control = {0};
        control.auto_rsp = ESP_GATT_AUTO_RSP;

        characteristic->service = service;
        characteristic->updated = false;
        characteristic_node_t *characteristic_node = malloc(sizeof(characteristic_node_t));
        characteristic_node->characteristic = characteristic;
        LIST_INSERT_HEAD(&profile->characteristics, characteristic_node, entries);

        esp_err_t err = esp_ble_gatts_add_char(service->handle, &characteristic->uuid, perm, property, &char_val, &control);
        if (err != 0) {
            LIST_REMOVE(characteristic_node, entries);
            free(characteristic_node);
            ESP_LOGE("bluetooth", "add char failed, error code =%x", err);
            return mp_bt_esp_errno(err);
        }
        // Wait for ESP_GATTS_ADD_CHAR_EVT
        xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
        if (mp_bt_call_status != 0) {
            LIST_REMOVE(characteristic_node, entries);
            free(characteristic_node);
            return mp_bt_status_errno();
        }
        
        LIST_INIT(&characteristic_node->descriptors);
        //Add descriptors
        mp_obj_list_t *descriptors = (mp_obj_list_t *)characteristic->descriptors;
        if (descriptors != mp_const_none && descriptors->len > 0) {
            for (size_t j = 0; j < descriptors->len; j++) {
                mp_bt_characteristic_t *descriptor = descriptors->items[j];

                perm = 0;
                perm |= (descriptor->flags & MP_BLE_FLAG_READ) ? ESP_GATT_PERM_READ : 0;
                perm |= (descriptor->flags & MP_BLE_FLAG_WRITE) ? ESP_GATT_PERM_WRITE : 0;

                esp_attr_value_t descr_val = {0};
                descr_val.attr_max_len = 2;
                descr_val.attr_len = 2;
                descr_val.attr_value = calloc(2, sizeof(uint8_t));

                esp_attr_control_t control = {0};
                control.auto_rsp = ESP_GATT_AUTO_RSP;

                descriptor->service = service;
                descriptor->updated = false;
                characteristic_node_t *descriptor_node = malloc(sizeof(characteristic_node_t));
                descriptor_node->characteristic = descriptor;
                LIST_INSERT_HEAD(&characteristic_node->descriptors, descriptor_node, entries);

                err = esp_ble_gatts_add_char_descr(service->handle, &descriptor->uuid, perm, &descr_val, &control);

                free(descr_val.attr_value);

                if (err != 0) {
                    LIST_REMOVE(descriptor_node, entries);
                    free(descriptor_node);
                    ESP_LOGE("bluetooth", "add char descr failed, error code =%x", err);
                    return mp_bt_esp_errno(err);
                }

                // Wait for ESP_GATTS_ADD_CHAR_EVT
                xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
                if (mp_bt_call_status != 0) {
                    LIST_REMOVE(descriptor_node, entries);
                    free(descriptor_node);
                    return mp_bt_status_errno();
                }
            }
        }
    }

    return 0;
}

int mp_bt_characteristic_value_set(mp_bt_characteristic_handle_t handle, const void *value, size_t value_len) {
    esp_err_t err = esp_ble_gatts_set_attr_value(handle, value_len, value);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    // Wait for ESP_GATTS_SET_ATTR_VAL_EVT
    xSemaphoreTake(mp_bt_call_complete, portMAX_DELAY);
    return mp_bt_status_errno();
}

int mp_bt_characteristic_notify(mp_bt_characteristic_handle_t handle, const void *value, size_t value_len) {
    if (global_is_connected) {
        if (value_len <= (global_mtu_size - 3)) {
            uint8_t *notify_data = (uint8_t *)malloc(value_len * sizeof(uint8_t));
            if(notify_data == NULL) {
                ESP_LOGE("bluetooth", "%s malloc.2 failed\n", __func__);
                return MP_EPERM;
            }
            memcpy(notify_data, value, value_len);
            esp_err_t err = esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, handle, value_len, notify_data, false);
            if (err != 0) {
                return mp_bt_esp_errno(err);
            }
            vTaskDelay(20 / portTICK_PERIOD_MS);
        } else {
            uint8_t total_num = 0;
            uint8_t current_num = 0;
            uint8_t *ntf_value_p = NULL;

            if ((value_len % (global_mtu_size - 7)) == 0) {
                total_num = value_len / (global_mtu_size - 7);
            } else {
                total_num = value_len / (global_mtu_size - 7) + 1;
            }
            current_num = 1;
            ntf_value_p = (uint8_t *)malloc((global_mtu_size - 3) * sizeof(uint8_t));
            if(ntf_value_p == NULL) {
                ESP_LOGE("bluetooth", "%s malloc.2 failed\n", __func__);
                return MP_EPERM;
            }
            while (current_num <= total_num) {
                esp_err_t err = 0;
                if (current_num < total_num) {
                    ntf_value_p[0] = '#';
                    ntf_value_p[1] = '#';
                    ntf_value_p[2] = total_num;
                    ntf_value_p[3] = current_num;
                    memcpy(ntf_value_p + 4, value + (current_num - 1) * (global_mtu_size - 7), (global_mtu_size - 7));
                    err = esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, handle, (global_mtu_size - 3), ntf_value_p, false);
                }
                else if (current_num == total_num) {
                    ntf_value_p[0] = '#';
                    ntf_value_p[1] = '#';
                    ntf_value_p[2] = total_num;
                    ntf_value_p[3] = current_num;
                    memcpy(ntf_value_p + 4, value + (current_num - 1) * (global_mtu_size - 7), (value_len - (current_num - 1) * (global_mtu_size - 7)));
                    err = esp_ble_gatts_send_indicate(global_gatts_if, global_conn_id, handle, (value_len - (current_num - 1) * (global_mtu_size - 7) + 4), ntf_value_p, false);
                }
                if (err != 0) {
                    return mp_bt_esp_errno(err);
                }
                vTaskDelay(20 / portTICK_PERIOD_MS);
                current_num++;
            }
            free(ntf_value_p);
        }
    }
    return mp_bt_status_errno();
}

int mp_bt_characteristic_value_get(mp_bt_characteristic_handle_t handle, void **value, size_t *value_len) {
    uint16_t bt_len;
    const uint8_t *bt_ptr;
    esp_err_t err = esp_ble_gatts_get_attr_value(handle, &bt_len, &bt_ptr);
    if (err != 0) {
        return mp_bt_esp_errno(err);
    }
    if (*value_len > bt_len) {
        // Copy up to *value_len bytes.
        *value_len = bt_len;
    }
    *value = malloc(*value_len);
    memcpy(*value, bt_ptr, *value_len);
    return 0;
}

void mp_bt_characteristic_value_wait(mp_bt_characteristic_t *characteristic) {
    if (characteristic->updated) {
        uint8_t *data = NULL;
        size_t value_len = MP_BT_MAX_ATTR_SIZE;
        int errno_ = mp_bt_characteristic_value_get(characteristic->value_handle, (void **)&data, &value_len);
        if (errno_ != 0) {
            mp_raise_OSError(errno_);
        }
        if (data != NULL) {
            mp_call_function_2(characteristic->callback, characteristic, mp_obj_new_bytes(data, value_len));
            free(data);
        }
        characteristic->updated = false;
    }
}

// Parse a UUID object from the caller.
void mp_bt_parse_uuid(mp_obj_t obj, mp_bt_uuid_t *uuid) {
    if (MP_OBJ_IS_SMALL_INT(obj) && MP_OBJ_SMALL_INT_VALUE(obj) == (uint32_t)(uint16_t)MP_OBJ_SMALL_INT_VALUE(obj)) {
        // Integer fits inside 16 bits, assume it's a standard UUID.
        uuid->len = ESP_UUID_LEN_16;
        uuid->uuid.uuid16 = MP_OBJ_SMALL_INT_VALUE(obj);
    } else if (mp_obj_is_str(obj)) {
        // Guessing this is a 128-bit (proprietary) UUID.
        uuid->len = ESP_UUID_LEN_128;
        mp_bt_parse_uuid_str(obj, &uuid->uuid.uuid128[0]);
    } else {
        mp_raise_ValueError("cannot parse UUID");
    }
}

// Format a UUID object to be returned from a .uuid() call.
mp_obj_t mp_bt_format_uuid(mp_bt_uuid_t *uuid) {
    switch (uuid->len) {
    case ESP_UUID_LEN_16:
        return MP_OBJ_NEW_SMALL_INT(uuid->uuid.uuid16);
    case ESP_UUID_LEN_128:
        return mp_bt_format_uuid_str(uuid->uuid.uuid128);
    default:
        return mp_const_none;
    }
}

// Event callbacks. Most API calls generate an event here to report the
// result.
STATIC void mp_bt_gap_callback(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            mp_bt_call_status = param->adv_data_raw_cmpl.status;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            mp_bt_call_status = param->scan_rsp_data_raw_cmpl.status;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            mp_bt_call_status = param->adv_start_cmpl.status;
            // May return an error (queue full) when called from
            // mp_bt_gatts_callback, but that's OK.
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            break;
        default:
            ESP_LOGI("bluetooth", "GAP: unknown event: %d", event);
            break;
    }
}

STATIC void mp_bt_gatts_profile_callback(profile_node_t *profile, esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI("bluetooth", "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
            global_conn_id = param->connect.conn_id;
            global_gatts_if = gatts_if;
            global_is_connected = true;
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            // restart advertisement
            ESP_LOGI("bluetooth", "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
            global_is_connected = false;
            mp_bt_advertise_start_internal();
            break;
        case ESP_GATTS_REG_EVT:
            mp_bt_call_status = param->reg.status;
            profile->service->gatts_if = gatts_if;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GATTS_CREATE_EVT:
            // Service created.
            ESP_LOGI("bluetooth", "CREATE_SERVICE_EVT, status 0x%x,  service_handle %d", param->create.status, param->create.service_handle);
            mp_bt_call_status = param->create.status;
            profile->service->handle = param->create.service_handle;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GATTS_START_EVT:
            // Service started.
            ESP_LOGI("bluetooth", "SERVICE_START_EVT, status 0x%x, service_handle %d", param->start.status, param->start.service_handle);
            mp_bt_call_status = param->start.status;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GATTS_ADD_CHAR_EVT:{
            // Characteristic added.
            ESP_LOGI("bluetooth", "ADD_CHAR_EVT, status 0x%x,  attr_handle %d, service_handle %d", param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            mp_bt_call_status = param->add_char.status;
            characteristic_node_t *chr_node = LIST_FIRST(&profile->characteristics);
            chr_node->characteristic->value_handle = param->add_char.attr_handle;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        }
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI("bluetooth", "ADD_DESCR_EVT, status 0x%x, attr_handle %d, service_handle %d", param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
            mp_bt_call_status = param->add_char_descr.status;
            characteristic_node_t *chr_node = LIST_FIRST(&profile->characteristics);
            characteristic_node_t *descr_node = LIST_FIRST(&chr_node->descriptors);
            descr_node->characteristic->value_handle = param->add_char_descr.attr_handle;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        case ESP_GATTS_SET_ATTR_VAL_EVT:{
            // Characteristic value set by application.
            mp_bt_call_status = param->set_attr_val.status;
            xSemaphoreGive(mp_bt_call_complete);
            break;
        }
        case ESP_GATTS_READ_EVT:
            // Characteristic value read by connected device.
            ESP_LOGI("bluetooth", "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
            break;
        case ESP_GATTS_WRITE_EVT: {
            // Characteristic value written by connected device.
            mp_bt_call_result.attr_handle = param->write.handle;
            characteristic_node_t *node = _mp_bt_find_char_node(profile, param->write.handle);
            if (node) {
                if (!param->write.is_prep) {
                    node->characteristic->updated = true;
                } else {
                    profile->prepared = node;
                }
            }
            break;
        }
        case ESP_GATTS_EXEC_WRITE_EVT: {
            if(param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC) {
                if (profile->prepared) {
                    profile->prepared->characteristic->updated = true;
                    profile->prepared = NULL;
                }
            }
            xSemaphoreGive(mp_bt_call_complete);
            break;
        }
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI("bluetooth", "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            global_mtu_size = param->mtu.mtu;
            break;
        case ESP_GATTS_CONF_EVT:
            break;
        default:
            ESP_LOGI("bluetooth", "GATTS: unknown event: %d", event);
            break;
    }
}

STATIC void mp_bt_gatts_callback(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            profile_node_t *profile;
            LIST_FOREACH(profile, &profiles, entries)
                profile->service->gatts_if = gatts_if;
        } else {
            ESP_LOGI("bluetooth", "Reg app failed, app_id %04x, status 0x%x",param->reg.app_id, param->reg.status);
            return;
        }
    }

    do {
        profile_node_t *profile;
        LIST_FOREACH(profile, &profiles, entries)
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == profile->service->gatts_if) {
                mp_bt_gatts_profile_callback(profile, event, gatts_if, param);
            }
    } while (0);
}

#endif // MICROPY_PY_BLUETOOTH
