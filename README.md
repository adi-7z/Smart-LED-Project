# smart light with ambient controls
* uses depth and ambient light of the environment to adjust it's light automatically.
* can be controlled & monitored using a web dashboard
<p align="center">
  <img src="https://github.com/user-attachments/assets/0efab13f-efeb-4c4e-ab5c-47215bc28e41" height="300">
  <img src="https://github.com/user-attachments/assets/76285999-8184-449a-80c2-7a9d8cdccfd3" height="300">
  <img src="https://github.com/user-attachments/assets/e1c51044-9d8b-4343-a6c2-c54d6be3b43e" height="300">

</p>
## features
* real time graphs for system temperature, ambient light, and room depth.

* **AUTO mode:** adjusts brightness smoothly & dynamically. you can set at what light condition the smart light will automatically turn off, with the lux threshold.
* **MANUAL mode:** custom colors and brightness settings.

## hardware
* **MCU:** esp32 dev board
* **light:** 8x8 NeoPixel Matrix (WS2812B RGB LED)
* **Sensors:**
    * BH1750 (I2C Light Sensor)
    * HC-SR04 (Ultrasonic Distance Sensor)
<p align="center">
  <video src="https://github.com/user-attachments/assets/ba0ce071-8637-4d28-9fb7-0a61c7c75cac" width="300" controls></video>
</p>


## pin configuration
| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **I2C SDA** | GPIO 21 | BH1750 |
| **I2C SCL** | GPIO 22 | BH1750 |
| **BH Pwr** | GPIO 27 | Used as VCC |
| **BH Gnd** | GPIO 26 | Used as GND |
| **Trig** | GPIO 13 | Ultrasonic |
| **Echo** | GPIO 14 | Ultrasonic |
| **Matrix** | GPIO 15 | DIN |

## dependencies
* `ESPAsyncWebServer` & `AsyncTCP`
* `Adafruit_NeoMatrix` & `Adafruit_NeoPixel`
* `BH1750`
* `LittleFS` 
