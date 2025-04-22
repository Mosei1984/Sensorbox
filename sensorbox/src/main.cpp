#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME680.h>
#include <Adafruit_CCS811.h>
#include <TinyGPS++.h>

// Pin Definitions - optimized for ESP32 DevKit V4
#define MQ135_PIN 34  // Analog input for MQ-135 (Air quality)
#define MQ2_PIN 32    // Analog input for MQ-2 (Smoke, LPG, CO)

// GPS pins
#define GPS_RX 16  // ESP32 RX pin connected to GPS TX
#define GPS_TX 17  // ESP32 TX pin connected to GPS RX

// TFT Display Pins
#define TFT_CS   5   // Chip select pin
#define TFT_RST  4   // Reset pin
#define TFT_DC   2   // Data/Command pin
#define TFT_MOSI 23  // SPI MOSI pin
#define TFT_SCLK 18  // SPI Clock pin

// Number of samples for averaging
#define NUM_SAMPLES 500

// Alert thresholds (adjust as needed)
#define MQ135_THRESHOLD 2000
#define MQ2_THRESHOLD 2000
#define CCS811_CO2_THRESHOLD 2000
#define CCS811_TVOC_THRESHOLD 400

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0
#define WHITE    0xFFFF
#define ORANGE   0xFD20

// Objects
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_BME680 bme;
Adafruit_CCS811 ccs;
TinyGPSPlus gps;
HardwareSerial GPSSerial(1); // Use UART1 for GPS

// Variables to store sensor readings
int mq135Value = 0;
int mq2Value = 0;
int temperature = 0;
int humidity = 0;
int pressure = 0;
int gasResistance = 0;
int eco2 = 0;
int tvoc = 0;
bool alertState = false;

// GPS variables
float latitude = 0.0;
float longitude = 0.0;
int satellites = 0;
bool gpsValid = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(100);
  Serial.println("Air Quality Monitoring System Starting...");
  
  // Initialize GPS serial
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS module initialized");

  // Initialize I2C - using default ESP32 pins (21 and 22)
  Wire.begin();

  // Initialize analog sensors
  pinMode(MQ135_PIN, INPUT);
  pinMode(MQ2_PIN, INPUT);

  // Initialize TFT display
  tft.initR(INITR_BLACKTAB); // Initialize ST7735S chip, black tab
  tft.setRotation(3); // Landscape orientation
  tft.fillScreen(BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.println("Initializing...");

  // Initialize BME680
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    tft.fillScreen(BLACK);
    tft.setCursor(5, 20);
    tft.setTextColor(RED);
    tft.println("BME680 not found!");
    delay(2000);
  } else {
    // Set up BME680 oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }

  // Initialize CCS811
  if(!ccs.begin()) {
    Serial.println("Failed to start CCS811 sensor! Check wiring.");
    tft.fillScreen(BLACK);
    tft.setCursor(5, 20);
    tft.setTextColor(RED);
    tft.println("CCS811 not found!");
    delay(2000);
  }

  // Allow sensors to warm up for better readings
  tft.fillScreen(BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(YELLOW);
  tft.println("Sensors warming up...");
  tft.setCursor(5, 40);
  tft.println("Please wait 1 minute");
  
  for (int i = 60; i > 0; i--) {
    tft.fillRect(5, 60, 150, 20, BLACK);
    tft.setCursor(5, 60);
    tft.print("Time remaining: ");
    tft.print(i);
    tft.print("s");
    delay(1000);
  }
}

void checkAlerts() {
  alertState = false;
  
  if (mq135Value > MQ135_THRESHOLD) {
    Serial.println("ALERT: High MQ135 reading - possible air contamination!");
    alertState = true;
  }
  
  if (mq2Value > MQ2_THRESHOLD) {
    Serial.println("ALERT: High MQ2 reading - smoke or flammable gas detected!");
    alertState = true;
  }
  
  if (eco2 > CCS811_CO2_THRESHOLD) {
    Serial.println("ALERT: High CO2 levels!");
    alertState = true;
  }
  
  if (tvoc > CCS811_TVOC_THRESHOLD) {
    Serial.println("ALERT: High TVOC levels!");
    alertState = true;
  }
}

void readGPS() {
  // Process GPS data if available
  while (GPSSerial.available() > 0) {
    if (gps.encode(GPSSerial.read())) {
      // If we have a new valid position
      if (gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
        gpsValid = true;
      } else {
        gpsValid = false;
      }
      
      // Get satellite count
      if (gps.satellites.isValid()) {
        satellites = gps.satellites.value();
      }
    }
  }
  
  // Check if GPS is getting data
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS data received: check wiring");
  }
}

void readSensors() {
  // Read MQ series sensors - take average of 500 readings
  long mq135Sum = 0;
  long mq2Sum = 0;
  
  tft.fillScreen(BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(YELLOW);
  tft.println("Taking readings...");
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    // Update progress every 10 samples
    if (i % 10 == 0) {
      tft.fillRect(5, 40, 150, 10, BLACK);
      tft.setCursor(5, 40);
      tft.print("Progress: ");
      tft.print(i * 100 / NUM_SAMPLES);
      tft.print("%");
    }
    
    mq135Sum += analogRead(MQ135_PIN);
    mq2Sum += analogRead(MQ2_PIN);
    
    // Process GPS data while taking readings
    while (GPSSerial.available() > 0) {
      gps.encode(GPSSerial.read());
    }
    
    delay(10); // Small delay between readings
  }
  
  // Calculate average
  mq135Value = round(mq135Sum / NUM_SAMPLES);
  mq2Value = round(mq2Sum / NUM_SAMPLES);
  
  // Read BME680
  if (bme.performReading()) {
    temperature = round(bme.temperature);
    pressure = round(bme.pressure / 100.0); // convert to hPa
    humidity = round(bme.humidity);
    gasResistance = round(bme.gas_resistance / 1000.0); // convert to kOhms
  } else {
    Serial.println("Failed to perform BME680 reading");
  }
  
  // Read CCS811
  if (ccs.available()) {
    if (ccs.readData()) {
      eco2 = ccs.geteCO2();
      tvoc = ccs.getTVOC();
      
      // Set environmental data for better accuracy
      ccs.setEnvironmentalData(humidity, temperature);
    } else {
      Serial.println("CCS811 error!");
    }
  }
  
  // Get GPS data
  readGPS();
}

// Function to format GPS coordinates for display
String formatGPS(float position) {
  // Convert to degrees, minutes, seconds format
  int degrees = (int)position;
  float minutes = (position - degrees) * 60;
  
  String formatted = String(degrees) + "° " + String(minutes, 4) + "'";
  return formatted;
}

void updateDisplay() {
  tft.fillScreen(BLACK);
  
  // If in alert state, display warning
  if (alertState) {
    tft.fillScreen(RED);
    tft.setCursor(25, 40);
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.println("ALERT!");
    tft.setTextSize(1);
    tft.setCursor(15, 70);
    tft.println("Check Serial Monitor");
    delay(1000);
    tft.fillScreen(BLACK);
  }
  
  // Display title
  tft.setCursor(5, 5);
  tft.setTextSize(1);
  tft.setTextColor(CYAN);
  tft.println("Workshop Air Quality Monitor");
  
  // Section 1: Environmental data
  tft.setCursor(5, 15);
  tft.setTextColor(GREEN);
  tft.println("Environment:");
  
  tft.setTextColor(WHITE);
  tft.setCursor(5, 25);
  tft.print("Temp: ");
  tft.print(temperature);
  tft.println(" C");
  
  tft.setCursor(5, 35);
  tft.print("Hum: ");
  tft.print(humidity);
  tft.println(" %");
  
  tft.setCursor(5, 45);
  tft.print("Press: ");
  tft.print(pressure);
  tft.println(" hPa");
  
  // Section 2: Gas sensors
  tft.setCursor(5, 60);
  tft.setTextColor(YELLOW);
  tft.println("Gas Sensors (avg of 500):");
  
  tft.setTextColor(WHITE);
  tft.setCursor(5, 70);
  tft.print("MQ135: ");
  tft.println(mq135Value);
  
  tft.setCursor(5, 80);
  tft.print("MQ2: ");
  tft.println(mq2Value);
  
  // Section 3: Air Quality
  tft.setCursor(5, 95);
  tft.setTextColor(MAGENTA);
  tft.println("Air Quality:");
  
  tft.setTextColor(WHITE);
  tft.setCursor(5, 105);
  tft.print("CO2: ");
  tft.print(eco2);
  tft.println(" ppm");
  
  tft.setCursor(5, 115);
  tft.print("TVOC: ");
  tft.print(tvoc);
  tft.println(" ppb");
  
  tft.setCursor(90, 70);
  tft.print("Gas: ");
  tft.print(gasResistance);
  tft.println(" kO");
  
  // GPS Section
  tft.setCursor(5, 130);
  tft.setTextColor(BLUE);
  tft.println("GPS Location:");
  
  tft.setTextColor(WHITE);
  if (gpsValid) {
    tft.setCursor(5, 140);
    tft.print("Lat: ");
    tft.print(latitude, 6);
    
    tft.setCursor(5, 150);
    tft.print("Lon: ");
    tft.print(longitude, 6);
  } else {
    tft.setCursor(5, 140);
    tft.println("Searching satellites...");
  }
  
  // Satellite count
  tft.setCursor(90, 140);
  tft.print("Sats: ");
  tft.println(satellites);
}

void loop() {
  readSensors();
  checkAlerts();
  updateDisplay();
  
  // Detailed information via Serial
  Serial.println("-------- SENSOR READINGS --------");
  Serial.println("MQ-135 (avg of 500): " + String(mq135Value));
  Serial.println("MQ-2 (avg of 500): " + String(mq2Value));
  Serial.println("Temperature: " + String(temperature) + " C");
  Serial.println("Humidity: " + String(humidity) + " %");
  Serial.println("Pressure: " + String(pressure) + " hPa");
  Serial.println("Gas Resistance: " + String(gasResistance) + " kOhms");
  Serial.println("eCO2: " + String(eco2) + " ppm");
  Serial.println("TVOC: " + String(tvoc) + " ppb");
  
  // GPS information
  Serial.println("-------- GPS DATA --------");
  if (gpsValid) {
    Serial.println("GPS Fix: Valid");
    Serial.println("Latitude: " + String(latitude, 6));
    Serial.println("Longitude: " + String(longitude, 6));
  } else {
    Serial.println("GPS Fix: Searching...");
  }
  Serial.println("Satellites: " + String(satellites));
  Serial.println("---------------------------------");
  
  delay(30000); // Update every 30 seconds since it takes time to gather 500 samples
}

