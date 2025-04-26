# Dual-Board Air Quality and GPS Monitoring System

## Overview

This project implements a portable air quality monitoring system with GPS functionality using a dual-board architecture. An Arduino Nano collects data from analog sensors, while an ESP32 processes this data along with readings from digital sensors, GPS location, and displays everything on a TFT screen. The system now includes WiFi connectivity and a web interface for remote monitoring and configuration.

## Features

- **Real-time air quality monitoring** with multiple sensors
- **GPS location tracking**
- **TFT display** for on-device data visualization
- **WiFi connectivity** for remote monitoring
- **Web interface** for configuration and data visualization
- **High-accuracy readings** using multi-sample averaging
- **Alert system** for dangerous air quality conditions
- **Serial output** for detailed logging and debugging
- **Distributed processing** across two microcontrollers

## Hardware Requirements

### Arduino Nano Components
- Arduino Nano microcontroller
- MQ-2 gas sensor (smoke, LPG, CO)
- MQ-135 gas sensor (air quality, NH3, NOx, alcohol, benzene, smoke, CO2)
- LDR light sensor
- Microphone for sound level detection
- 10kΩ resistor (for LDR circuit)

### ESP32 Components
- ESP32 development board (AZ-Delivery DevKit V4 recommended)
- ST7735 TFT display (160x128)
- BME680 environmental sensor (temperature, humidity, pressure, gas)
- CCS811 air quality sensor (eCO2, TVOC)
- GY-GPS6MV2 uBlox NEO-6M GPS module
- Breadboard and jumper wires
- Power supply (USB or battery)

## Wiring Diagram

### Arduino Nano Pin Connections

| Component | Pin | Nano Pin |
|-----------|-----|----------|
| MQ-2 | AO | A0 |
|      | VCC | 5V |
|      | GND | GND |
| MQ-135 | AO | A1 |
|        | VCC | 5V |
|        | GND | GND |
| LDR | Signal | A2 |
|     | Other terminal | 5V (with 10kΩ pull-down resistor to GND) |
| Microphone | AO | A4 |
|           | VCC | 5V |
|           | GND | GND |
| ESP32 Communication | TX | D10 |
|                    | RX | D9 (not used in current implementation) |

### ESP32 Pin Connections

| Component | Pin | ESP32 Pin |
|-----------|-----|-----------|
| ST7735 TFT | CS | 5 |
|           | DC | 2 |
|           | MOSI | 23 (default SPI) |
|           | SCLK | 18 (default SPI) |
|           | VCC | 3.3V |
|           | GND | GND |
| BME680 & CCS811 | SDA | 21 (default I2C) |
|                | SCL | 22 (default I2C) |
|                | VCC | 3.3V |
|                | GND | GND |
| GPS Module | TX | 16 (ESP32 RX) |
|           | RX | 17 (ESP32 TX) |
|           | VCC | 3.3V |
|           | GND | GND |
| Nano Communication | RX | 14 (receives from Nano) |
|                   | TX | 12 (not used in current implementation) |

## Software Architecture

The system uses a state machine approach on both microcontrollers:

### Arduino Nano (Nanomain.cpp)
- Waits for a "READY" handshake from the ESP32
- Samples MQ-2, MQ-135, and LDR sensors (100 samples averaged)
- Measures sound level using peak-to-peak voltage
- Calculates lux from LDR readings
- Sends all data to the ESP32
- Waits for the next sampling cycle

### ESP32 (main.cpp)
- Initializes all sensors and the display
- Sets up WiFi connectivity using WiFiManager for easy configuration
- Initializes SPIFFS for web interface files
- Configures AsyncWebServer for remote monitoring
- Sends "READY" handshake to the Nano
- Collects data from BME680, CCS811, and GPS
- Receives sensor data from the Nano
- Processes and displays all sensor readings
- Implements alert system for dangerous readings
- Serves web interface for remote monitoring and configuration
- Stores data in JSON format for web access

## Software Requirements

### Libraries for Arduino Nano
- Arduino core
- SoftwareSerial

### Libraries for ESP32
- Adafruit GFX Library
- Adafruit ST7735 Library
- Adafruit BME680 Library
- Adafruit CCS811 Library
- TinyGPS++ Library
- Arduino Wire Library
- EspSoftwareSerial
- WiFi
- AsyncTCP
- ESPAsyncWebServer
- SPIFFS
- ArduinoJson
- WiFiManager

## Development Environment

This project is designed to be built with PlatformIO, which handles the dual-board configuration.

### platformio.ini configuration:

```ini
[env:nano]
platform = atmelavr
board = nanoatmega328
framework = arduino
build_src_filter = +<Nanomain.cpp> -<main.cpp>
lib_deps =
    SoftwareSerial

[env:esp32]
platform = espressif32
board = az-delivery-devkit-v4
framework = arduino
build_src_filter = +<main.cpp> -<Nanomain.cpp>
lib_deps =
    adafruit/Adafruit SSD1306@^2.5.7
    adafruit/Adafruit GFX Library@^1.11.5
    adafruit/Adafruit BME680 Library@^2.0.2
    adafruit/Adafruit CCS811 Library@^1.1.1
    mikalhart/TinyGPSPlus@^1.0.3
    adafruit/Adafruit ST7735 and ST7789 Library@^1.11.0
    plerup/EspSoftwareSerial@^8.2.0
    me-no-dev/AsyncTCP
    me-no-dev/ESP Async WebServer
    bblanchon/ArduinoJson
    tzapu/WiFiManager
    Wire
monitor_speed = 115200
```

## Installation

### PlatformIO (Recommended)
1. Install Visual Studio Code
2. Install the PlatformIO extension

3. Clone this repository:
```bash
git clone https://github.com/yourusername/dual-board-air-quality-monitor.git
```

4. Open the project folder in VS Code
5. Connect your Arduino Nano and upload the Nano code:
```bash
pio run -e nano -t upload
```

6. Connect your ESP32 and upload the ESP32 code:
```bash
pio run -e esp32 -t upload
```

7. Upload the SPIFFS data for the web interface:
```bash
pio run -e esp32 -t uploadfs
```

## Communication Protocol

The Arduino Nano and ESP32 communicate via a simple serial protocol:

- ESP32 sends "READY" to initiate data collection
- Nano collects sensor data and sends it in the format:
  ```
  MQ2:[value]
  MQ135:[value]
  Lux:[value]
  DB:[value]
  ```
- ESP32 processes this data along with its own sensor readings.

## Web Interface

The system provides a web interface accessible via WiFi:

1. On first boot, the ESP32 creates a WiFi access point named "AirQualityMonitor"
2. Connect to this network with your smartphone or computer
3. A captive portal will open for WiFi configuration
4. After configuration, the device will connect to your network
5. Access the web interface by navigating to the ESP32's IP address
6. The interface provides:
   - Real-time sensor readings
   - Configuration options

## Usage

### First Run and Calibration
1. Power on both the Arduino Nano and ESP32
2. Configure WiFi using the captive portal
3. The system will enter a warm-up period for the sensors
4. After warm-up, the device will begin taking sensor readings
5. Allow the GPS module time to acquire satellites (may take several minutes on first use)

### Interpreting the Display

The TFT display shows:
- Environmental data (temperature, humidity, pressure)
- Gas sensor readings (MQ-2, MQ-135)
- Light level (lux)
- Sound level (dB)
- Air quality (eCO2, TVOC)
- GPS location (latitude, longitude, satellite count)
- WiFi connection status

### Serial Monitoring
Connect to the ESP32's serial monitor at 115200 baud to view detailed sensor readings and diagnostic information.

## Troubleshooting Common Issues

- **No communication between boards**: Check the serial connections and make sure both boards are powered
- **No GPS fix**: Ensure the GPS module has a clear view of the sky
- **Sensor readings too high/low**: MQ sensors require a warm-up period of 24-48 hours for best results
- **Display not working**: Check SPI connections and pin definitions
- **WiFi connection issues**: Reset the WiFi configuration by holding the reset button for 10 seconds
- **Web interface not loading**: Check that SPIFFS was properly uploaded

## Contributing

Contributions to this project are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- Adafruit for their excellent sensor libraries
- TinyGPS++ for GPS parsing functionality
- The Arduino and ESP32 communities for their support and documentation
- WiFiManager developers for the easy WiFi configuration solution
- ESPAsyncWebServer team for the web server functionality
