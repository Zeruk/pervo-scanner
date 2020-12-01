#ifndef NETWOEK_H    // To make sure you don't declare the function more than once by including the header multiple times.
#define NETWOEK_H

// #include "protocol_examples_common.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"

// #include "lwip/apps/netbiosns.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "dns_server.h"



void net_initialise_all(void);


#endif