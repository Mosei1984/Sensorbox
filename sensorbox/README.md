# ESP32 Air Quality and GPS Monitoring System

## Overview
This project implements a portable air quality monitoring system with GPS functionality using an ESP32 microcontroller. The system collects environmental data (temperature, humidity, pressure), air quality metrics (CO2, TVOC, gas resistance), and location information to provide a comprehensive view of air quality with geospatial context.

![Project Image Placeholder](images/device_photo.jpg)

## Features
- Real-time air quality monitoring with multiple sensors
- GPS location tracking
- TFT display for on-device data visualization
- High-accuracy readings using 500-sample averaging
- Alert system for dangerous air quality conditions
- Serial output for detailed logging and debugging

## Hardware Requirements

### Components
- ESP32 development board (DevKit V4 recommended)
- ST7735 TFT display (160x128)
- BME680 environmental sensor (temperature, humidity, pressure, gas)
- CCS811 air quality sensor (eCO2, TVOC)
- MQ-135 gas sensor (air quality, NH3, NOx, alcohol, benzene, smoke, CO2)
- MQ-2 gas sensor (smoke, LPG, CO)
- GY-GPS6MV2 uBlox NEO-6M GPS module
- Breadboard and jumper wires
- Power supply (USB or battery)

### Wiring Diagram

#### Pin Connections
| Component | Pin | ESP32 Pin |
|-----------|-----|-----------|
| **ST7735 TFT** | CS | 5 |
|  | RST | 4 |
|  | DC | 2 |
|  | MOSI | 23 |
|  | SCLK | 18 |
|  | VCC | 3.3V |
|  | GND | GND |
| **BME680 & CCS811** | SDA | 21 |
|  | SCL | 22 |
|  | VCC | 3.3V |
|  | GND | GND |
| **MQ-135** | AO | 34 |
|  | VCC | 5V |
|  | GND | GND |
| **MQ-2** | AO | 32 |
|  | VCC | 5V |
|  | GND | GND |
| **GPS Module** | TX | 16 (ESP32 RX) |
|  | RX | 17 (ESP32 TX) |
|  | VCC | 3.3V |
|  | GND | GND |

## Software Requirements

### Libraries
- Adafruit GFX Library
- Adafruit ST7735 Library
- Adafruit BME680 Library
- Adafruit CCS811 Library
- TinyGPS++ Library
- Arduino Wire Library

### Development Environment
This project is designed to be built with PlatformIO, but can also be compiled with the Arduino IDE if the appropriate libraries are installed.

#### platformio.ini configuration
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
  adafruit/Adafruit GFX Library@^1.11.5
  adafruit/Adafruit ST7735 and ST7789 Library@^1.10.0
  adafruit/Adafruit BME680 Library@^2.0.2
  adafruit/Adafruit CCS811 Library@^1.1.1
  mikalhart/TinyGPSPlus@^1.0.3
  Wire
monitor_speed = 115200
```

## Installation

### PlatformIO (Recommended)
1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install the [PlatformIO extension](https://platformio.org/install/ide?install=vscode)
3. Clone this repository:
   ```bash
   git clone https://github.com/yourusername/esp32-air-quality-gps-monitor.git
   ```
4. Open the project folder in VS Code
5. Connect your ESP32 via USB
6. Click the PlatformIO build/upload button to compile and flash the code

### Arduino IDE
1. Install the [Arduino IDE](https://www.arduino.cc/en/software)
2. Install ESP32 board support following [these instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md)
3. Install all required libraries via the Library Manager
4. Copy the contents of src/main.cpp into a new Arduino sketch
5. Select the appropriate ESP32 board from Tools > Board
6. Connect your ESP32 via USB and upload the sketch

## Usage

### First Run and Calibration
1. Power on the device in a known clean air environment
2. The system will enter a warm-up period (60 seconds)
3. After warm-up, the device will begin taking sensor readings
4. The system takes 500 readings from each analog sensor to provide accurate averages
5. Allow the GPS module time to acquire satellites (may take several minutes on first use)

### Interpreting the Display
The TFT display is divided into sections:
- **Environment**: Temperature, humidity, and pressure readings
- **Gas Sensors**: MQ-135 and MQ-2 sensor readings (averaged over 500 samples)
- **Air Quality**: eCO2 and TVOC readings from the CCS811 sensor
- **GPS Location**: Current latitude, longitude, and satellite count

### Alert System
The system will display a red alert screen when dangerous levels are detected:
- High MQ-135 readings (possible air contamination)
- High MQ-2 readings (smoke or flammable gases)
- High CO2 levels
- High TVOC levels

Detailed alert information is output to the Serial monitor.

## Troubleshooting

### Common Issues
- **No GPS fix**: Ensure the GPS module has a clear view of the sky. GPS reception is poor indoors.
- **Sensor readings too high/low**: MQ sensors require a warm-up period of 24-48 hours for best results.
- **Display not working**: Check SPI connections and pin definitions.
- **Serial output garbled**: Ensure the monitor speed is set to 115200 baud.

### Serial Monitor
Connect to the serial monitor at 115200 baud to view detailed sensor readings and diagnostic information.

## Contributing
Contributions to this project are welcome! Please feel free to submit a Pull Request.

## License
This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments
- Adafruit for their excellent sensor libraries
- TinyGPS++ for GPS parsing functionality
- The ESP32 community for their support and documentation