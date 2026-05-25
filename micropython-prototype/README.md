# MicroPython Prototype

This folder contains the first MicroPython prototype of the radar.

It is kept as reference material. The Arduino C++ firmware in `../arduino-radar/` is the recommended version because it uses a background FreeRTOS task for network fetches, which keeps the OLED animation smoother.

Before running this prototype, edit the placeholders in `main.py`:

```python
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"

HOME_LAT = 51.500000
HOME_LON = -0.120000
HOME_PLACE = "HOME"
```
