# Smart LED Matrix & IoT Monitor

An ESP32-based 8x8 LED matrix controller with a reactive web dashboard. The system operates in two modes: **Manual** (web control) or **Auto** (adjusts based on ambient light and proximity).

## Features
* **Dual Mode Control:**
    * **Auto:** Fades brightness dynamically based on LUX readings and user distance.
    * **Manual:** Full RGB color selection and brightness control via web UI.
* **Web Dashboard:** Responsive interface (LittleFS) with real-time graphs for:
    * Internal Chip Temperature
    * Ambient Light (Lux)
    * Proximity (cm)
* **Soft Dimming:** Smooth transition algorithms for LED brightness.

## Hardware
* **MCU:** ESP32 Development Board
* **Display:** 8x8 NeoPixel RGB Matrix (WS2812B)
* **Sensors:** * BH1750 (I2C Light Sensor)
    * HC-SR04 (Ultrasonic Distance Sensor)
![led1](https://github.com/user-attachments/assets/0efab13f-efeb-4c4e-ab5c-47215bc28e41)
![led inside view](https://github.com/user-attachments/assets/76285999-8184-449a-80c2-7a9d8cdccfd3)

## Pinout Configuration
| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **I2C SDA** | GPIO 21 | BH1750 |
| **I2C SCL** | GPIO 22 | BH1750 |
| **BH Pwr** | GPIO 27 | Used as VCC |
| **BH Gnd** | GPIO 26 | Used as GND |
| **Trig** | GPIO 13 | Ultrasonic |
| **Echo** | GPIO 14 | Ultrasonic |
| **Matrix** | GPIO 15 | DIN |

## Dependencies
* `ESPAsyncWebServer` & `AsyncTCP`
* `Adafruit_NeoMatrix` & `Adafruit_NeoPixel`
* `BH1750`
* `LittleFS` 
