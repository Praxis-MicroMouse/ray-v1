#ifndef OTA_H
#define OTA_H

// Wireless link for the testing phase: the ESP32 hosts its own WiFi access
// point (no router/shared network needed) and the laptop connects directly
// to it. That link is then used for two things:
//   - ArduinoOTA, so `pio run -e esp32dev_ota -t upload` can reflash the
//     board over WiFi instead of USB.
//   - telemetry.h's DATA, lines, which telemetry_send() also broadcasts as
//     UDP once this AP is up (see TELEMETRY_UDP_PORT in telemetry.h).
//
// Connect the laptop's WiFi to OTA_AP_SSID/OTA_AP_PASSWORD below - the
// board is always reachable at OTA_AP_IP (192.168.4.1, the ESP32's default
// SoftAP address) once ota_init() has run.

#define OTA_AP_SSID "MicroMouse"
#define OTA_AP_PASSWORD "mmouse2026" // WPA2 needs >= 8 chars
#define OTA_HOSTNAME "micromouse"
#define OTA_AP_IP "192.168.4.1" // ESP32 SoftAP default - matches platformio.ini's upload_port

// Brings up the SoftAP and starts ArduinoOTA listening on it. Call once
// from setup(), before telemetry_init(). Logs the AP's IP over Serial so
// it's visible even without WiFi already connected.
void ota_init(void);

// Services pending OTA requests - must be called every loop() iteration
// (non-blocking; a no-op the rest of the time). No-op if ota_init() was
// never called.
void ota_handle(void);

#endif // OTA_H
