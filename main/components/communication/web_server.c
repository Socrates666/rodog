#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_timer.h"
// #include "esp_camera.h"
// #include "img_converters.h"
#include "esp_netif.h"
#include "lwip/ip_addr.h"      // IP地址定义
#include "lwip/ip4_addr.h"     // IPv4地址操作
#include "lwip/inet.h"         // 网络字节序转换
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_mac.h"
#include "web_server.h"
#include "web_page.h"
#include "leg.h"

static const char *TAG = "WEBSERVER";

// WiFi 配置
const char* AP_SSID = "WAVESHARE Robot";
const char* AP_PWD  = "1234567890";
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";

// 全局变量定义
int WIFIP_MODE;
esp_ip4_addr_t IP_ADDRESS;
uint8_t WIFI_MODE;
char MAC_ADDRESS[18] = {0};
int WIFI_RSSI = 0;

// HTTP 服务器句柄
static httpd_handle_t stream_httpd = NULL;
static httpd_handle_t camera_httpd = NULL;

// Camera pins for ESP-EYE
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     4
#define SIOD_GPIO_NUM     18
#define SIOC_GPIO_NUM     23
#define Y9_GPIO_NUM       36
#define Y8_GPIO_NUM       37
#define Y7_GPIO_NUM       38
#define Y6_GPIO_NUM       39
#define Y5_GPIO_NUM       35
#define Y4_GPIO_NUM       14
#define Y3_GPIO_NUM       13
#define Y2_GPIO_NUM       34
#define VSYNC_GPIO_NUM    5
#define HREF_GPIO_NUM     27
#define PCLK_GPIO_NUM     25



// WiFi 事件处理函数
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "STA started, connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "STA disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        IP_ADDRESS = event->ip_info.ip;
        WIFI_MODE = 2;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&IP_ADDRESS));
        ESP_LOGI(TAG, "RSSI: %d dBm", WIFI_RSSI);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Station connected, MAC: %02x:%02x:%02x:%02x:%02x:%02x, AID: %d",
                 event->mac[0], event->mac[1], event->mac[2],
                 event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
    }
}

// 获取 MAC 地址
void getMAC(void) {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(MAC_ADDRESS, sizeof(MAC_ADDRESS), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "MAC: %s", MAC_ADDRESS);
}

// 获取 IP 地址
void getIP(void) {
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
            IP_ADDRESS = ip_info.ip;
            ESP_LOGI(TAG, "STA IP: " IPSTR, IP2STR(&IP_ADDRESS));
        }
    }
}

// 获取 WiFi 状态
void getWifiStatus(void) {
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        WIFI_RSSI = ap_info.rssi;
        ESP_LOGD(TAG, "RSSI updated: %d dBm", WIFI_RSSI);
    }
}

// 设置 AP 模式
static void setAP(void) {
    ESP_LOGI(TAG, "Setting AP mode...");
    
    // 初始化 TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    wifi_config_t wifi_config = {
        .ap = {
            .ssid_len = strlen(AP_SSID),
            .channel = 1,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    
    strcpy((char*)wifi_config.ap.ssid, AP_SSID);
    strcpy((char*)wifi_config.ap.password, AP_PWD);
    
    if (strlen(AP_PWD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);
    esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    esp_netif_set_ip_info(ap_netif, &ip_info);
    
    IP_ADDRESS = ip_info.ip;
    WIFI_MODE = 1;
    
    ESP_LOGI(TAG, "AP mode started. SSID: %s, IP: " IPSTR, AP_SSID, IP2STR(&IP_ADDRESS));
}

// 设置 STA 模式
static void setSTA(void) {
    ESP_LOGI(TAG, "Setting STA mode...");
    
    // 初始化 TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    strcpy((char*)wifi_config.sta.ssid, STA_SSID);
    strcpy((char*)wifi_config.sta.password, STA_PWD);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    WIFI_MODE = 3; // 连接中状态
    ESP_LOGI(TAG, "STA mode started, connecting to %s...", STA_SSID);
}

// WiFi 初始化
void wifiInit(void) {
    // 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    WIFI_MODE = DEFAULT_WIFI_MODE;
    getMAC();
    
    if (WIFI_MODE == 1) {
        setAP();
    } else if (WIFI_MODE == 2) {
        setSTA();
    }
}

// 流媒体服务器相关定义
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

typedef struct {
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

// 视频流处理函数
// static esp_err_t stream_handler(httpd_req_t *req) {
//     camera_fb_t *fb = NULL;
//     esp_err_t res = ESP_OK;
//     size_t _jpg_buf_len = 0;
//     uint8_t *_jpg_buf = NULL;
//     char part_buf[64];
    
//     static int64_t last_frame = 0;
//     if (!last_frame) {
//         last_frame = esp_timer_get_time();
//     }
    
//     res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
//     if (res != ESP_OK) {
//         return res;
//     }
    
//     while (true) {
//         fb = esp_camera_fb_get();
//         if (!fb) {
//             ESP_LOGE(TAG, "Camera capture failed");
//             res = ESP_FAIL;
//         } else {
//             if (fb->format != PIXFORMAT_JPEG) {
//                 bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
//                 // esp_camera_fb_return(fb);
//                 fb = NULL;
//                 if (!jpeg_converted) {
//                     ESP_LOGE(TAG, "JPEG compression failed");
//                     res = ESP_FAIL;
//                 }
//             } else {
//                 _jpg_buf_len = fb->len;
//                 _jpg_buf = fb->buf;
//             }
//         }
        
//         if (res == ESP_OK) {
//             res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
//         }
//         if (res == ESP_OK) {
//             size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len);
//             res = httpd_resp_send_chunk(req, part_buf, hlen);
//         }
//         if (res == ESP_OK) {
//             res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
//         }
        
//         // if (fb) {
//         //     // esp_camera_fb_return(fb);
//         //     fb = NULL;
//         //     _jpg_buf = NULL;
//         // } else if (_jpg_buf) {
//         //     free(_jpg_buf);
//         //     _jpg_buf = NULL;
//         // }
        
//         if (res != ESP_OK) {
//             break;
//         }
        
//         int64_t fr_end = esp_timer_get_time();
//         int64_t frame_time = fr_end - last_frame;
//         last_frame = fr_end;
//         frame_time /= 1000;
        
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
    
//     last_frame = 0;
//     return res;
// }

// 命令处理函数
static esp_err_t cmd_handler(httpd_req_t *req) {
    char* buf;
    size_t buf_len;
    char variable[32] = {0};
    char value[32] = {0};
    char cmd[32] = {0};
    
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if (!buf) {
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
                httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK &&
                httpd_query_key_value(buf, "cmd", cmd, sizeof(cmd)) == ESP_OK) {
                // 参数解析成功
            } else {
                free(buf);
                httpd_resp_send_404(req);
                return ESP_FAIL;
            }
        } else {
            free(buf);
            httpd_resp_send_404(req);
            return ESP_FAIL;
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    
    int val = atoi(value);
    int cmdint = atoi(cmd);
    int res = 0;
    
    // 根据 URL 中的参数确定功能
    // if (!strcmp(variable, "framesize")) {
    //     ESP_LOGI(TAG, "framesize");
    //     if (s->pixformat == PIXFORMAT_JPEG) {
    //         res = s->set_framesize(s, (framesize_t)val);
    //     }
    // }
    // 功能控制
    if (!strcmp(variable, "funcMode")) {
        debugMode = 0;
        if (val == 1) {
            if (funcMode == 1) {
                funcMode = 0;
                ESP_LOGI(TAG, "Steady OFF");
            } else if (funcMode == 0) {
                funcMode = 1;
                ESP_LOGI(TAG, "Steady ON");
            }
        } else {
            funcMode = val;
            ESP_LOGI(TAG, "funcMode: %d", val);
        }
    }
    // 伺服调试模式
    else if (!strcmp(variable, "sconfig")) {
        debugMode = 1;
        funcMode = 0;
        servoDebug(val, cmdint);
        ESP_LOGI(TAG, "servo:%d position:%d MID:%d offset:%d", 
                val, CurrentPWM[val], ServoMiddlePWM[val], cmdint);
    }
    else if (!strcmp(variable, "sset")) {
        if (debugMode) {
            // servoConfigSave(val);
            ESP_LOGI(TAG, "SET servo:%d position:%d", val, ServoMiddlePWM[val]);
        } else {
            ESP_LOGW(TAG, "DebugMode = 0, servo config could not be saved.");
        }
    }
    // 移动控制
    else if (!strcmp(variable, "move")) {
        debugMode = 0;
        funcMode = 0;
        if (val == 1) {
            ESP_LOGI(TAG, "Forward");
            moveFB = 1;
        } else if (val == 2) {
            ESP_LOGI(TAG, "TurnLeft");
            moveLR = -1;
        } else if (val == 3) {
            ESP_LOGI(TAG, "FBStop");
            moveFB = 0;
        } else if (val == 4) {
            ESP_LOGI(TAG, "TurnRight");
            moveLR = 1;
        } else if (val == 5) {
            ESP_LOGI(TAG, "Backward");
            moveFB = -1;
        } else if (val == 6) {
            ESP_LOGI(TAG, "LRStop");
            moveLR = 0;
        }
    } else {
        ESP_LOGW(TAG, "Unknown variable: %s", variable);
        res = -1;
    }
    
    if (res) {
        return httpd_resp_send_500(req);
    }
    
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, NULL, 0);
}

// 主页处理函数
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

// 启动相机服务器
static void startCameraServer(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_handler,
        .user_ctx = NULL
    };
    
    httpd_uri_t cmd_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = cmd_handler,
        .user_ctx = NULL
    };
    
    // httpd_uri_t stream_uri = {
    //     .uri = "/stream",
    //     .method = HTTP_GET,
    //     .handler = stream_handler,
    //     .user_ctx = NULL
    // };
    
    ESP_LOGI(TAG, "Starting web server on port: %d", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
    } else {
        ESP_LOGE(TAG, "Failed to start camera HTTP server");
        return;
    }
    
    // config.server_port += 1;
    // config.ctrl_port += 1;
    // ESP_LOGI(TAG, "Starting stream server on port: %d", config.server_port);
    // if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    //     // httpd_register_uri_handler(stream_httpd, &stream_uri);
    // } else {
    //     ESP_LOGE(TAG, "Failed to start stream HTTP server");
    // }
}

// Web 服务器初始化
void webServerInit(void) {
    ESP_LOGI(TAG, "Initializing web server...");

    // 初始化 WiFi
    wifiInit();
    
    // 启动相机服务器
    vTaskDelay(pdMS_TO_TICKS(1000));
    startCameraServer();
    
    ESP_LOGI(TAG, "Web server initialization complete");
}