#ifndef OTA_H
#define OTA_H

// Two wireless-link modes, both feeding the same ArduinoOTA + telemetry.h
// UDP broadcast (see TELEMETRY_UDP_PORT in telemetry.h):
//
//   - ota_init() (AP mode): the ESP32 hosts its own WiFi access point (no
//     router/shared network needed) and the laptop connects directly to
//     it, at the fixed address OTA_AP_IP.
//   - ota_init_sta() (station mode): the ESP32 joins an existing WiFi
//     network instead (WIFI_STA_SSID/PASSWORD in wifi_credentials.h - copy
//     wifi_credentials.example.h and fill in your real network first) -
//     use this to get telemetry/OTA over the air on the same network your
//     laptop is already on, no separate AP to switch to. The board's IP is
//     DHCP-assigned by that network's router and printed over Serial once
//     connected; there's no fixed address to hardcode like OTA_AP_IP.
//
// Only call one of the two from setup(), not both.

#define OTA_AP_SSID "MicroMouse"
#define OTA_AP_PASSWORD "mmouse2026" // WPA2 needs >= 8 chars
#define OTA_HOSTNAME "micromouse"
#define OTA_AP_IP "192.168.4.1" // ESP32 SoftAP default - matches platformio.ini's upload_port

#define OTA_STA_CONNECT_TIMEOUT_MS 15000 // give up joining the network after this long

// Brings up the SoftAP and starts ArduinoOTA listening on it. Call once
// from setup(), before telemetry_init(). Logs the AP's IP over Serial so
// it's visible even without WiFi already connected.
void ota_init(void);

// Joins the WiFi network named in wifi_credentials.h (WIFI_STA_SSID/
// WIFI_STA_PASSWORD) and starts ArduinoOTA listening on it, same as
// ota_init() but station mode instead of AP mode. Call once from setup(),
// before telemetry_init(). Logs the assigned IP over Serial once
// connected; if the join times out (OTA_STA_CONNECT_TIMEOUT_MS), logs a
// failure and leaves OTA/telemetry off for the rest of the run (ota_handle()
// becomes a no-op) rather than blocking forever.
void ota_init_sta(void);

// Services pending OTA requests - must be called every loop() iteration
// (non-blocking; a no-op the rest of the time). No-op if neither ota_init()
// nor ota_init_sta() succeeded.
void ota_handle(void);

#endif // OTA_H
