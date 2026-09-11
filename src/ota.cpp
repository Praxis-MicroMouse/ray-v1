#include "ota.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

#include "wifi_credentials.h"

static bool s_ota_ready = false;

static void start_arduino_ota(void)
{
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
}

void ota_init(void)
{
    WiFi.mode(WIFI_AP);
    WiFi.softAP(OTA_AP_SSID, OTA_AP_PASSWORD);

    Serial.printf("[OTA] access point \"%s\" up (password \"%s\") - connect your\n",
                  OTA_AP_SSID, OTA_AP_PASSWORD);
    Serial.printf("[OTA] laptop's WiFi to it, board is at %s\n",
                  WiFi.softAPIP().toString().c_str());

    start_arduino_ota();

    Serial.println("[OTA] ready - flash wirelessly with: pio run -e esp32dev_ota -t upload");
}

void ota_init_sta(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.setHostname(OTA_HOSTNAME);
    WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);

    Serial.printf("[OTA] joining WiFi network \"%s\"", WIFI_STA_SSID);

    uint32_t start_ms = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - start_ms > OTA_STA_CONNECT_TIMEOUT_MS)
        {
            Serial.println();
            Serial.printf("[OTA] failed to join \"%s\" within %lums - OTA/telemetry off\n",
                          WIFI_STA_SSID, (unsigned long)OTA_STA_CONNECT_TIMEOUT_MS);
            return;
        }
        delay(250);
        Serial.print(".");
    }

    Serial.println();
    Serial.printf("[OTA] joined - board is at %s\n", WiFi.localIP().toString().c_str());

    start_arduino_ota();

    Serial.println("[OTA] ready - flash wirelessly with: pio run -e esp32dev_ota -t upload");
    Serial.println("[OTA] (update platformio.ini's esp32dev_ota upload_port to the IP above first)");
}

void ota_handle(void)
{
    if (s_ota_ready)
    {
        ArduinoOTA.handle();
    }
}
