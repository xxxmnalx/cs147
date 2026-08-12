#pragma once

// Copy this file to include/secrets.h and edit the values.
// Never commit real credentials to a public repository.

#define WIFI_SSID       "YOUR_WIFI_SSID"
#define WIFI_PASSWORD   "YOUR_WIFI_PASSWORD"

#define MQTT_HOST       "test.mosquitto.org"
#define MQTT_PORT       1883
#define MQTT_CLIENT_ID  "cs147-device-CHANGE-ME"
#define MQTT_TOPIC_BASE "cs147/demo"

#define OTA_HOSTNAME    "cs147-esp32"
#define OTA_PASSWORD    "change-me"
