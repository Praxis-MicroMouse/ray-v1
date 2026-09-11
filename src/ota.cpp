#include "ota.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

static bool s_ota_ready = false;

void ota_init(void)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(OTA_AP_SSID, OTA_AP_PASSWORD);

    Serial.printf("[OTA] access point \"%s\" up (password \"%s\") - connect your\n",
                  OTA_AP_SSID, OTA_AP_PASSWORD);
    Serial.printf("[OTA] laptop's WiFi to it, board is at %s\n",
                  WiFi.softAPIP().toString().c_str());

    ArduinoOTA.setHostname(OTA_HOSTNAME);

    ArduinoOTA.onStart([]() {
        Serial.println("[OTA] flash starting...");
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("[OTA] flash complete, rebooting");
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[OTA] error [%u]\n", (unsigned)error);
    });

    ArduinoOTA.begin();
    s_ota_ready = true;

    Serial.println("[OTA] ready - flash wirelessly with: pio run -e esp32dev_ota -t upload");
}

void ota_handle(void)
{
    if (s_ota_ready)
    {
        ArduinoOTA.handle();
    }
}
