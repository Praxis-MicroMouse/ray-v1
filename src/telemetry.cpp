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

// Picks the broadcast address to reach whatever laptop is on the ESP32's
// current link, whichever WiFi mode ota.cpp brought up: the fixed SoftAP
// subnet if we're an access point, or the joined network's own subnet
// (computed from our assigned IP + netmask) if we're a station. Returns
// false if neither link is actually up (nobody to reach).
static bool get_broadcast_address(IPAddress *out)
{
    if (WiFi.getMode() == WIFI_AP)
    {
        if (WiFi.softAPgetStationNum() == 0)
        {
            return false; // AP up but nobody connected yet
        }
        *out = IPAddress(192, 168, 4, 255);
        return true;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        uint32_t ip = (uint32_t)WiFi.localIP();
        uint32_t mask = (uint32_t)WiFi.subnetMask();
        *out = IPAddress(ip | ~mask);
        return true;
    }

    return false;
}

static void send_line(const char *line, size_t len)
{
    Serial.write((const uint8_t *)line, len);

    if (!s_udp_ready)
    {
        return;
    }

    IPAddress broadcast;
    if (!get_broadcast_address(&broadcast))
    {
        return;
    }

    s_udp.beginPacket(broadcast, TELEMETRY_UDP_PORT);
    s_udp.write((const uint8_t *)line, len);
    s_udp.endPacket();
}

void telemetry_send(const sensor_reading_t *reading)
{
    char line[64];
    int len = snprintf(line, sizeof(line), "DATA,%lu,%u,%u,%u\n",
                        (unsigned long)millis(),
                        reading->front_mm,
                        reading->right_mm,
                        reading->left_mm);

    send_line(line, (size_t)len);
}

void telemetry_send_line(const char *line)
{
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "%s\n", line);
    if (len < 0)
    {
        return;
    }
    if ((size_t)len >= sizeof(buf))
    {
        len = sizeof(buf) - 1; // truncated - still send what fits
    }

    send_line(buf, (size_t)len);
}
