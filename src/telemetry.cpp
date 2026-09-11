#include "telemetry.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

static WiFiUDP s_udp;
static bool s_udp_ready = false;

void telemetry_init(void)
{
    s_udp_ready = true;
}

void telemetry_send(const sensor_reading_t *reading)
{
    char line[64];
    int len = snprintf(line, sizeof(line), "DATA,%lu,%u,%u,%u\n",
                        (unsigned long)millis(),
                        reading->front_mm,
                        reading->right_mm,
                        reading->left_mm);

    Serial.write((const uint8_t *)line, len);

    // SoftAP subnet broadcast (192.168.4.x) - reaches the laptop connected
    // to the AP without needing to know its exact IP. Skipped when nobody's
    // connected so this doesn't spam the radio while running on USB alone.
    if (s_udp_ready && WiFi.softAPgetStationNum() > 0)
    {
        IPAddress broadcast(192, 168, 4, 255);
        s_udp.beginPacket(broadcast, TELEMETRY_UDP_PORT);
        s_udp.write((const uint8_t *)line, len);
        s_udp.endPacket();
    }
}
