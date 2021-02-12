#ifndef _CONFIG_H
#define _CONFIG_H

#include <IPAddress.h>
#include "HvacControl.h"

#define ENABLE_OTA
#define ENABLE_MDNS
#define SERIAL_BAUD 115200
#define DEFAULT_SSID "your_ssid_here"
#define DEFAULT_PASSWORD "your_password_here"
#define CLOCK_TIMEZONE -4
#define CONFIG_FILE_PATH "/config.json"
#define CHECK_WIFI_INTERVAL 60000
#define CHECK_MQTT_INTERVAL 120000
#define CLOCK_SYNC_INTERVAL 3600000
#define TEMP_READ_INTERVAL 2000
#define HVAC_CYCLE_INTERVAL 5000
#define MQTT_TOPIC_STATUS "espstat/status"
#define MQTT_TOPIC_CONTROL "espstat/control"
#define MQTT_BROKER "your_mqtt_host_here"
#define MQTT_PORT 1883
#define DEFAULT_HOST_NAME "ESPSTAT-MAIN"
#define DEFAULT_SET_POINT 70
#ifdef ENABLE_OTA
    #include <ArduinoOTA.h>
    #define OTA_HOST_PORT 8266                     // The OTA updater port.
    #define OTA_PASSWORD "your_ota_password_here"  // The OTA updater password.
#endif
IPAddress defaultIp(192, 168, 0, 230);                 // The default static host IP.
IPAddress defaultGw(192, 168, 0, 1);                   // The default static gateway IP.
IPAddress defaultSm(255, 255, 255, 0);                 // The default static subnet mask.
IPAddress defaultDns(defaultGw);

typedef struct {
    // Network stuff
    String hostname;
    String ssid;
    String password;
    IPAddress ip;
    IPAddress gw;
    IPAddress sm;
    IPAddress dns;
    bool useDhcp;

    uint8_t clockTimezone;

    // MQTT stuff
    String mqttTopicStatus;
    String mqttTopicControl;
    String mqttBroker;
    String mqttUsername;
    String mqttPassword;
    uint16_t mqttPort;

    // OTA stuff
    uint16_t otaPort;
    String otaPassword;

	// HVAC settings
	HvacMode currentMode;
	float setPoint;
} config_t;

#endif