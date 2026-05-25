# Arduino C++ Firmware

Main firmware for Heltec Air Radar.

## What It Does

- Draws a portrait radar sweep on the built-in OLED
- Fetches nearby aircraft in a background FreeRTOS task
- Projects aircraft movement locally between fetches
- Uses the `PRG` button for aircraft details

## Controls

- Short press `PRG`: show the nearest aircraft, then cycle through aircraft
- Long press `PRG`: return to radar

## Configuration

Edit the top of `arduino-radar.ino` before upload:

```cpp
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

constexpr double HOME_LAT = 51.500000;
constexpr double HOME_LON = -0.120000;
const char* HOME_PLACE = "HOME";
```

Use broad or approximate coordinates in public forks.
