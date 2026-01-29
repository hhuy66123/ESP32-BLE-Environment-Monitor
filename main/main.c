#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "dht11.h"
#include "sdkconfig.h"

#define GATTS_TAG "GATTS_DEMO"
#define LDR_DO_PIN GPIO_NUM_4 // Chân digital output của cảm biến ánh sáng LDR 
#define DHT_GPIO GPIO_NUM_5 // Chân bạn nối với DATA của DHT11
#define PA_GPIO GPIO_NUM_2   // GPIO2
uint8_t flag_send_ble = 0;

#define WAKEUP_INTERVAL_SEC 15 // Thời gian giãn giữa 2 lần wakeup khi dùng timer wakeup
#define WAKEUP_GPIO GPIO_NUM_3  // Chân GPIO dùng để đánh thức ESP32 từ chế độ deep sleep

///Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void DHT_task(void *arg);

#define GATTS_SERVICE_UUID_TEST_A   0x181A    //* UUID để BLE chuẩn nhận diện dịch vụ A là Environmental Sensing */
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     10       //* Số handle cho service A, bao gồm service, characteristic, descriptor phải thay đổi khi thay đổi bất kì 1 trong 3 thành phần trên */

#define GATTS_SERVICE_UUID_TEST_B   0x00EE
#define GATTS_CHAR_UUID_TEST_B      0xEE01
#define GATTS_DESCR_UUID_TEST_B     0x2222
#define GATTS_NUM_HANDLE_TEST_B     4

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

#define TEST_MANUFACTURER_DATA_LEN  17
#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40
#define PREPARE_BUF_MAX_SIZE 1024

#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

static uint8_t char1_str[] = {0x11,0x22,0x33};
static uint16_t descr_value = 0x0;
static uint16_t local_mtu = 23;
static uint8_t char_value_read[CONFIG_EXAMPLE_CHAR_READ_DATA_LEN] = {0xDE,0xED,0xBE,0xEF}; // Giá trị gửi cho client khi nó muốn đọc đặc tính (characteristic) của server
static char test_device_name[ESP_BLE_ADV_NAME_LEN_MAX] = "BLE Group 7 Device";  // Tên thiết bị BLE hiển thị khi quét
volatile int64_t t_start = 0, t_end = 0;
volatile int64_t t_start_latency = 0, t_end_latency = 0;
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,  //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,  //second uuid, 32bit, [12], [13], [14], [15] is the value
};

static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .p_manufacturer_data =  NULL, 
    .p_service_data = NULL,
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0, 
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;

    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;

    // --- Temperature ---
    uint16_t char_handle_temp;
    esp_bt_uuid_t char_uuid_temp;

    // --- Humidity ---
    uint16_t char_handle_humi;
    esp_bt_uuid_t char_uuid_humi;

    // --- Light ---
    uint16_t char_handle_light;
    esp_bt_uuid_t char_uuid_light;

    // --- 1 descriptor dùng chung cho notify ---
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

uint16_t conn_id_global = 0;     // lưu connection ID của client BLE
uint8_t  notify_enabled = 0;     // trạng thái bật/tắt notify

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Temperature */
esp_bt_uuid_t char_uuid_temp = { .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = 0x2A6E} };
static const char temp_init[] = "25.0";
esp_attr_value_t temp_val = {
    .attr_max_len = 8,
    .attr_len = sizeof(temp_init) - 1,
    .attr_value = (uint8_t *)temp_init,
};

/* Humidity */
esp_bt_uuid_t char_uuid_humi = { .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = 0x2A6F} };
static const char humi_init[] = "83.0";
esp_attr_value_t humi_val = {
    .attr_max_len = 8,
    .attr_len = sizeof(humi_init) - 1,
    .attr_value = (uint8_t *)humi_init,
};

/* Illuminance */
esp_bt_uuid_t char_uuid_light = { .len = ESP_UUID_LEN_16, .uuid = {.uuid16 = 0x2A77} };
static const char light_init[] = "350";
esp_attr_value_t light_val = {
    .attr_max_len = 8,
    .attr_len = sizeof(light_init) - 1,
    .attr_value = (uint8_t *)light_init,
};

/* CCCD uuid (dùng chung) */
esp_bt_uuid_t cccd_uuid = {
    .len = ESP_UUID_LEN_16,
    .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
};

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

// Presentation Format Descriptor cho Temperature (0x2A6E)
static uint8_t temp_format[] = {
    0x0E,       // format: IEEE-11073 float (0x0E)
    0xFF,       // exponent: -1 (0xFF = -1)
    0x5A, 0x27, // unit: Temperature Celsius (0x275A)
    0x01, 0x00, // namespace: Bluetooth SIG (0x0001)
    0x00, 0x00  // description: 0
};

void Wakeup_conf(void)
{
    // Cấu hình chân GPIO để đánh thức
    gpio_reset_pin(WAKEUP_GPIO);
    gpio_set_direction(WAKEUP_GPIO, GPIO_MODE_INPUT);
    gpio_pulldown_dis(WAKEUP_GPIO);
    gpio_pullup_en(WAKEUP_GPIO);
    // Cấu hình chế độ wakeup EXT0: 1 chân, mức logic thấp
    esp_deep_sleep_enable_gpio_wakeup((1ULL << WAKEUP_GPIO), ESP_GPIO_WAKEUP_GPIO_HIGH); // đổi 2
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: // Sự kiện hoàn tất bắt đầu quảng bá chung của thiết bị cho tất cả các service
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed, status %d", param->adv_start_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising start successfully");
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed, status %d", param->adv_stop_cmpl.status);
            break;
        }
        ESP_LOGI(GATTS_TAG, "Advertising stop successfully");
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         ESP_LOGI(GATTS_TAG, "Connection params update, status %d, conn_int %d, latency %d, timeout %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "Packet length update, status %d, rx %d, tx %d",
                  param->pkt_data_length_cmpl.status,
                  param->pkt_data_length_cmpl.params.rx_len,
                  param->pkt_data_length_cmpl.params.tx_len);
        break;
    default:
        break;
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC){
        ESP_LOG_BUFFER_HEX(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TAG,"Prepare write cancel");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "GATT server register, status %d, app_id %d, gatts_if %d", param->reg.status, param->reg.app_id, gatts_if);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(test_device_name); // đặt tên thiết bị BLE
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data); // Cấu hình dữ liệu quảng bá
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);  // tạo serive cho BLE
        break;
    case ESP_GATTS_READ_EVT: { // Sự kiện đọc đặc tính (characteristic) từ thiết bị client
        esp_gatt_rsp_t rsp = {0};

        if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].char_handle_temp) {
            const char *p = temp_init;
            memcpy(rsp.attr_value.value, p, strlen(p));
            rsp.attr_value.len = strlen(p);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            return;
        }
        if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].char_handle_humi) {
            const char *p = humi_init;
            memcpy(rsp.attr_value.value, p, strlen(p));
            rsp.attr_value.len = strlen(p);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            return;
        }
        if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].char_handle_light) {
            const char *p = light_init;
            memcpy(rsp.attr_value.value, p, strlen(p));
            rsp.attr_value.len = strlen(p);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            return;
        }

        /* Nếu client đọc CCCD (thường trả 2 bytes) */
        if (param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].descr_handle ||
            param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].descr_handle ||
            param->read.handle == gl_profile_tab[PROFILE_A_APP_ID].descr_handle) {
            memcpy(rsp.attr_value.value, &descr_value, 2);
            rsp.attr_value.len = 2;
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            return;
        }
        break;

    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG,"Execute write");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "MTU exchange, MTU %d", param->mtu.mtu);
        local_mtu = param->mtu.mtu;
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:  // Sự kiện tạo dịch vụ, case đầu tiên được gọi sau khi đăng ký GATT server, Quảng bá BLE (advertising) chỉ nên bắt đầu sau khi service đã được tạo xong
    ESP_LOGI(GATTS_TAG, "Service create, handle %d", param->create.service_handle);
    gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
    esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);

            /* Add Temperature */
    esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                        &char_uuid_temp,
                        ESP_GATT_PERM_READ,
                        ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                        &temp_val, NULL);
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {   // Sự kiện thêm đặc tính (characteristic) vào dịch vụ, được gọi sau khi hàm esp_ble_gatts_add_char() được thực hiện thành công
        static int char_idx = 0;
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, idx=%d, attr_handle=%d", char_idx, param->add_char.attr_handle);

        if (char_idx == 0) {
            gl_profile_tab[PROFILE_A_APP_ID].char_handle_temp = param->add_char.attr_handle;
        } else if (char_idx == 1) {
            gl_profile_tab[PROFILE_A_APP_ID].char_handle_humi = param->add_char.attr_handle;
            ESP_LOGE(GATTS_TAG, "ADD_CHAR_EVT HUMIDITY ");
        } else if (char_idx == 2) {
            gl_profile_tab[PROFILE_A_APP_ID].char_handle_light = param->add_char.attr_handle;
            ESP_LOGE(GATTS_TAG, "ADD_CHAR_EVT LIGHT ");
            char_idx = -1; 
        }

        /* Thêm CCCD cho characteristic vừa thêm */
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(
            gl_profile_tab[PROFILE_A_APP_ID].service_handle,
            &cccd_uuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            NULL,
            NULL);
        if (add_descr_ret != ESP_OK) {
            ESP_LOGE(GATTS_TAG, "add_char_descr failed: %s", esp_err_to_name(add_descr_ret));
        }
        char_idx++;

        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: // Sự kiện thêm mô tả (descriptor) vào đặc tính, được gọi sau khi hàm esp_ble_gatts_add_char_descr() được thực hiện thành công
        static int descr_idx = 0;
        ESP_LOGI(GATTS_TAG, "ADD_CHAR_DESCR_EVT idx=%d handle=%d", descr_idx, param->add_char_descr.attr_handle);

        if (descr_idx == 0) {
            gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
                        /* Add Humidity */
            esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                    &char_uuid_humi,
                                    ESP_GATT_PERM_READ,
                                    ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                    &humi_val, NULL);
            ESP_LOGE(GATTS_TAG, "esp_ble_gatts_add_char HUMIDITY ");
        } else if (descr_idx == 1) {
            gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
                /* Add Illuminance */
            esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle,
                                &char_uuid_light,
                                ESP_GATT_PERM_READ,
                                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                &light_val, NULL);
            ESP_LOGE(GATTS_TAG, "esp_ble_gatts_add_char LIGHT ");
        } else if (descr_idx == 2) {
            gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
            descr_idx = -1; 
        }
        descr_idx++;

        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:  // Sự kiện bắt đầu dịch vụ, được gọi sau khi hàm esp_ble_gatts_start_service() được gọi thành công
        ESP_LOGI(GATTS_TAG, "Service start, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {   // Sự kiện kết nối, được gọi khi một thiết bị client kết nối đến GATT server
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "Connected, conn_id %u, remote "ESP_BD_ADDR_STR"",
                 param->connect.conn_id, ESP_BD_ADDR_HEX(param->connect.remote_bda));

        conn_id_global = param->connect.conn_id;  // Lưu connection ID của client BLE để gửi notify của nhiệt độ, độ ẩm, ánh sáng

        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: // Sự kiện ngắt kết nối, được gọi khi thiết bị client ngắt kết nối khỏi GATT server
        ESP_LOGI(GATTS_TAG, "Disconnected, remote "ESP_BD_ADDR_STR", reason 0x%02x",
                 ESP_BD_ADDR_HEX(param->disconnect.remote_bda), param->disconnect.reason);
        printf("Disconnected! Going to sleep...\n");
        conn_id_global = 0;  // reset connection ID khi ngắt kết nối
        esp_sleep_enable_timer_wakeup(WAKEUP_INTERVAL_SEC * 1000000ULL); // Cấu hình deep sleep timer
        t_start = 0;
        esp_deep_sleep_enable_gpio_wakeup((1ULL << WAKEUP_GPIO), ESP_GPIO_WAKEUP_GPIO_HIGH); // cho phép GPIO đánh thức ESP32 từ chế độ deep sleep  //1
        printf("Entering deep sleep for %d s\n", WAKEUP_INTERVAL_SEC);
        t_start_latency = esp_timer_get_time();
        esp_deep_sleep_start(); // Bắt đầu chế độ deep sleep
        break;
    case ESP_GATTS_CONF_EVT:
        t_end = esp_timer_get_time();
        ESP_LOGI("RTT", "RTT = %d us", t_end - t_start);
        t_start = esp_timer_get_time();
        ESP_LOGI(GATTS_TAG, "Confirm receive, status %d, attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK){
            ESP_LOG_BUFFER_HEX(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    
    gpio_config_t pa_conf = 
    {
        .pin_bit_mask = 1ULL << PA_GPIO, // chọn GPIO2
        .mode = GPIO_MODE_OUTPUT,          // output
        .pull_up_en = GPIO_PULLUP_DISABLE, // không bật pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE     // không dùng ngắt
    };
    gpio_config(&pa_conf);
    gpio_set_level(PA_GPIO, 1); // Set GPIO2 HIGH để bật PA

    esp_err_t ret;
    ESP_LOGE("MAIN", "START MAIN");
    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    

    #if CONFIG_EXAMPLE_CI_PIPELINE_ID
    memcpy(test_device_name, esp_bluedroid_get_example_name(), ESP_BLE_ADV_NAME_LEN_MAX);
    #endif

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();  // tạo dữ liệu cấu hình để khởi tạo BT controller bằng hàm dưới
    ret = esp_bt_controller_init(&bt_cfg);  // khởi tạo BT controller với cấu hình đã tạo
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); // Thiết lập công suất truyền BLE ở mức cao nhất (9 dBm) để tăng phạm vi hoạt động

    /***********************************Xong lớp vật lí********************************************** */

    ret = esp_bluedroid_init();   // khởi tạo host stack (bao gồm GAP, GATT, SMP…).
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();  //bật stack để sẵn sàng hoạt động. Lúc này Bluetooth stack đã sẵn sàng, nhưng ứng dụng chưa có logic gì cả.
    if (ret) {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    // Note: Avoid performing time-consuming operations within callback functions.
    ret = esp_ble_gatts_register_callback(gatts_event_handler);  // đăng ký callback xử lý các sự kiện GATT server.
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);  // đăng ký callback xử lý các sự kiện GAP.
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }
    Wakeup_conf();
    vTaskDelay(pdMS_TO_TICKS(2000)); // Đợi 2 giây cho BLE ổn định
     
    t_end_latency = esp_timer_get_time();
    ESP_LOGI("WakeupLatency", "Time = %d us", t_end_latency - t_start_latency);


    xTaskCreate(DHT_task, "DHT_task", 1024 * 4, NULL, 5, NULL);
}

void float_to_ieee11073(float value, uint8_t *buf) {
    int32_t mantissa = (int32_t)(value * 10.0f);  // nhân 10 để giữ 1 chữ số thập phân
    int8_t exponent = -1;  // luôn là -1 cho °C với 1 decimal
    // Xử lý số âm
    if (mantissa < 0) {
        mantissa = -mantissa;
        mantissa = ~mantissa + 1;  // two's complement
    }
    // Ghi 3 byte mantissa (little-endian)
    buf[0] = mantissa & 0xFF;
    buf[1] = (mantissa >> 8) & 0xFF;
    buf[2] = (mantissa >> 16) & 0xFF;
    buf[3] = exponent;  // 0xFF
}


void DHT_task(void *arg)
{
    DHT11_init(DHT_GPIO);
    gpio_reset_pin(DHT_GPIO);
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);  
    gpio_pulldown_dis(DHT_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 1);
    
    // 1. Cấu hình GPIO làm input light
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LDR_DO_PIN), // chọn chân
        .mode = GPIO_MODE_INPUT,               // chế độ input
        .pull_up_en = GPIO_PULLUP_DISABLE,     // kéo lên
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // không kéo xuống
        .intr_type = GPIO_INTR_DISABLE         // không dùng ngắt
    };
    gpio_config(&io_conf);

    while (1)
    {
        
        while(DHT11_read().temperature == -1);
        printf("Temperature is %d \n", DHT11_read().temperature);
        printf("Humidity is %d\n", DHT11_read().humidity);
        printf("Status code is %d\n", DHT11_read().status);

        int temp = DHT11_read().temperature * 10;  // ví dụ: 30
        int hum = DHT11_read().humidity * 10;  // ví dụ: 30
        uint8_t state = gpio_get_level(LDR_DO_PIN);
        printf("Light is %d \n", state);

        uint8_t data1[4];
        uint8_t data2[4];
        float_to_ieee11073((float)temp, data1);
        float_to_ieee11073((float)hum, data2);
        gpio_set_level(PA_GPIO, 1); // Bật PA trước khi gửi dữ liệu BLE
        //ESP_LOGI("BLE", "Sending IEEE-11073: 0x%02X 0x%02X 0x%02X 0x%02X", data[0], data[1], data[2], data[3]);
        t_start = esp_timer_get_time();
        esp_ble_gatts_send_indicate(
            gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
            conn_id_global,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle_temp,
            4,                    // gửi đúng 4 byte
            data1,
            false);
    
        
        
        esp_ble_gatts_send_indicate(
            gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
            conn_id_global,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle_humi,
            4,                    // gửi đúng 4 byte
            data2,
            false);
    
        
        esp_ble_gatts_send_indicate(
            gl_profile_tab[PROFILE_A_APP_ID].gatts_if,
            conn_id_global,
            gl_profile_tab[PROFILE_A_APP_ID].char_handle_light,
            1,                    // gửi đúng 4 byte
            &state,
            false
        );
        gpio_set_level(PA_GPIO, 0); // Bật PA trước khi gửi dữ liệu BLE
        esp_ble_gatts_close(gl_profile_tab[PROFILE_A_APP_ID].gatts_if,conn_id_global); // Đóng kết nối BLE sau khi gửi dữ liệu
        vTaskDelay(pdMS_TO_TICKS(1000)); // Đợi 5 giây trước khi đọc lại
    }
}