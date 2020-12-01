/*
Here lays all network initializations
*/

#include "lwip/apps/netbiosns.h"
#include "protocol_examples_common.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "mdns.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define NETWORK_MDNS_HOST_NAME "pervoscanner"

#define NETWORK_ESP_WIFI_SSID      "pervoscanner"
#define NETWORK_ESP_WIFI_PASS      "pervoscanner"
#define NETWORK_ESP_WIFI_CHANNEL   CONFIG_ESP_WIFI_CHANNEL
#define NETWORK_MAX_STA_CONN       CONFIG_ESP_MAX_STA_CONN

static const char *TAG = "Network";

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = NETWORK_ESP_WIFI_SSID,
            .ssid_len = strlen(NETWORK_ESP_WIFI_SSID),
            .channel = NETWORK_ESP_WIFI_CHANNEL,
            .password = NETWORK_ESP_WIFI_PASS,
            .max_connection = NETWORK_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    if (strlen(NETWORK_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             NETWORK_ESP_WIFI_SSID, NETWORK_ESP_WIFI_PASS, NETWORK_ESP_WIFI_CHANNEL);
}

void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
}


esp_err_t start_rest_server(const char *base_path);

static void net_initialise_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    mdns_hostname_set(NETWORK_MDNS_HOST_NAME);
    mdns_instance_name_set(NETWORK_MDNS_HOST_NAME);

    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0); // just server
}

static void net_initialise(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    netbiosns_init();
    netbiosns_set_name(NETWORK_MDNS_HOST_NAME);
    
}