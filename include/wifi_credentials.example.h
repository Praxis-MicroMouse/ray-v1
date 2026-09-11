#ifndef WIFI_CREDENTIALS_H
#define WIFI_CREDENTIALS_H

// Copy this file to wifi_credentials.h (gitignored - see .gitignore) and
// fill in your real network's SSID/password below. wifi_credentials.h is
// what ota.cpp's ota_init_sta() actually includes; this .example file is
// just the committed template so the real credentials never hit git
// history.

#define WIFI_STA_SSID "your-network-name"
#define WIFI_STA_PASSWORD "your-network-password"

#endif // WIFI_CREDENTIALS_H
