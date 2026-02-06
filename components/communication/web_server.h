#ifndef WEBSERVER_H
#define WEBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_netif.h"
#include "board.h"
// WiFi 配置
#define DEFAULT_WIFI_MODE 1



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