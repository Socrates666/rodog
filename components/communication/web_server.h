#ifndef WEBSERVER_H
#define WEBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "esp_netif.h"
#include "board.h"
// WiFi 配置
#if defined(CONFIG_COMM_WEBSERVER_WIFI_MODE_AP)
#define DEFAULT_WIFI_MODE 1
#elif defined(CONFIG_COMM_WEBSERVER_WIFI_MODE_STA)
#define DEFAULT_WIFI_MODE 2
#else
// Fallback for older sdkconfig or when Kconfig wasn't loaded.
#define DEFAULT_WIFI_MODE 1
#endif



// 函数声明
void getMAC(void);
bool getIP(char *out, size_t out_len);
void getWifiStatus(void);
void wifiInit(void);
void webServerInit(void);

#ifdef __cplusplus
}
#endif

#endif // WEBSERVER_H