#ifndef WEBSERVER_H
#define WEBSERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_netif.h"
#include "components/control/leg.h"

// WiFi 配置
#define DEFAULT_WIFI_MODE 1



// 函数声明
void getMAC(void);
void getIP(void);
void getWifiStatus(void);
void wifiInit(void);
void webServerInit(void);

// 从其他模块导入的函数
// void servoConfigSave(uint8_t activeServo);
void initPosAll(void);
void middlePosAll(void);

#ifdef __cplusplus
}
#endif

#endif // WEBSERVER_H