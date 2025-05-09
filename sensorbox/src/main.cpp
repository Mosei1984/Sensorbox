#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BME680.h>
#include <Adafruit_CCS811.h>
#include <TinyGPSPlus.h>
#include <math.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
#include <WebTemplates.h>

// ——— Pin-Definitionen ——————————————————————————————
#define GPS_RX        16    // UART2 RX ← GPS TX
#define GPS_TX        17    // UART2 TX → GPS RX
#define NANO_RX_PIN   14    // SoftwareSerial RX (Nano TX)
#define NANO_TX_PIN   12    // SoftwareSerial TX (unused)
#define TFT_CS         5
#define TFT_DC         2
#define TFT_RST        4
#define TFT_MOSI      23
#define TFT_SCLK      18
#define MQ135_THRESHOLD       2000
#define MQ2_THRESHOLD         3500
#define CCS811_CO2_THRESHOLD  2000
#define CCS811_TVOC_THRESHOLD 400

#define SUMMER_TIME_OFFSET    2     // UTC+2
static const uint32_t WARM_DURATION_MS = 60UL*1000UL;  // 60 s
static const uint16_t HEADER_HEIGHT    = 16;
static const uint16_t LINE_HEIGHT      =  8;


// ——— Colors ————————————————————————————————————————
#define BLACK   0x0000
#define WHITE   0xFFFF
#define CYAN    0x07FF
#define YELLOW  0xFFE0
#define RED     0xF800

// ——— Objects ———————————————————————————————————————
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_BME680  bme;
Adafruit_CCS811  ccs;
TinyGPSPlus      gps;
SoftwareSerial nanoSerial(NANO_RX_PIN, NANO_TX_PIN);
AsyncWebServer server(80);
WiFiManager wifiManager;
// ——— Sensordaten ——————————————————————————————
float temperature = 0.0f;
float humidity = 0.0f;
float pressure = 0.0f;
float altitude = 0.0f;
uint16_t eco2 = 0;
uint16_t tvoc = 0;
float mq2 = 0.0f;
float mq135 = 0.0f;
float lux = 0.0f;
float db = 0.0f;
float vibMag = 0.0f;
float latitude = 0.0f;
float longitude = 0.0f;
int satellites = 0;
// Hardware-Serial für GPS (UART2)
#define gpsSerial Serial2

// ——— Sensor-Storage —————————————————————————————————
int   sensor_mq135 = 0, sensor_mq2 = 0;
float sensor_lux   = 0,   sensor_db = 0;
bool  alertState   = false;
float airQualityScore = 0.0f;

// ——— Umwelt- & GPS-Storage —————————————————————————
float gasResistance;
String alarmReason = "";
bool  gpsValid = false;
uint8_t gpsHour, gpsMinute, gpsSecond, gpsDay, gpsMonth;
uint16_t gpsYear;
bool warmingDone = false, dataReceived = false;
float vibX, vibY, vibZ;// ——— FSM Zustände —————————————————————————————————
enum ESPState {
  SEND_READY,
  READ_NANO,
  READ_GPS,
  READ_ENV,
  CHECK_ALERTS,
  UPDATE_DISPLAY,
  IDLE
};
ESPState state = SEND_READY;uint32_t stateTs = 0;

// ——— Prototypen ———————————————————————————————————
void readFromNano();
void readGPS();
void readEnvSensors();
void checkAlerts();
void updateDisplay();
void initCCS();

void setupWebServer();  // Forward declaration

void setup(){
  Serial.begin(115200);
  nanoSerial.begin(9600);
  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX, GPS_TX);

  Wire.begin();
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
   // Initialize SPIFFS
   if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS mount failed");
  }
  
  // Initialize WiFi
  if (wifiManager.begin()) {
    Serial.print("WiFi ");
    if (wifiManager.isAPMode()) {
      Serial.print("AP: ");
      Serial.println(wifiManager.getSSID());
    } else {
      Serial.print("STA: ");
      Serial.println(wifiManager.getSSID());
    }
    Serial.print("IP: ");
    Serial.println(wifiManager.getIP());
  }

  // Set up the web server - make sure this is called
  setupWebServer();

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(3);
  tft.fillScreen(BLACK);

  if (bme.begin()) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320,150);
  }
  initCCS();

  stateTs = millis();
  Serial.println("[ESP] Setup complete, entering FSM");
}void initCCS(){
  if (!ccs.begin()) {
    while(!ccs.begin());
  }
  ccs.setDriveMode(CCS811_DRIVE_MODE_1SEC);
}

void loop(){
  static ESPState lastState = (ESPState)-1;
  uint32_t now = millis();

  // — Debug: State transition logging —
  if(state != lastState){
    Serial.print("[ESP] New State → ");
    switch(state){
      case SEND_READY:    Serial.println("SEND_READY");    break;
      case READ_NANO:     Serial.println("READ_NANO");     break;
      case READ_GPS:      Serial.println("READ_GPS");      break;
      case READ_ENV:      Serial.println("READ_ENV");      break;
      case CHECK_ALERTS:  Serial.println("CHECK_ALERTS");  break;
      case UPDATE_DISPLAY:Serial.println("UPDATE_DISPLAY");break;
      case IDLE:          Serial.println("IDLE");          break;
    }
    lastState = state;
  }

  switch(state) {
    case SEND_READY:
      // you can still send one initial READY, but real handshake happens below
      nanoSerial.println("READY");
      dataReceived = false;
      state = READ_NANO;
      break;

    case READ_NANO:
      nanoSerial.println("READY");
      static bool readyMessageSent = false;
      
      if (!readyMessageSent) {
        nanoSerial.println("READY");
        readyMessageSent = true;
      }
      
      readFromNano();
      
      if(dataReceived){
        Serial.println("[ESP] ← Complete Nano block received");
        dataReceived = false;
        readyMessageSent = false;  // Reset for next cycle
        state = READ_GPS;
      }
      break;

    case READ_GPS:
      readGPS();
      state = READ_ENV;
      break;

    case READ_ENV:{
      static uint32_t lastRemSec = UINT32_MAX;
      uint32_t elapsed = now - stateTs;
      if(!warmingDone && elapsed < WARM_DURATION_MS){
        uint32_t remSec = (WARM_DURATION_MS - elapsed + 999)/1000;
        if(remSec != lastRemSec){
          lastRemSec = remSec;
          // TFT countdown
          tft.fillScreen(BLACK);
          tft.setTextSize(1); tft.setTextColor(YELLOW);
          tft.setCursor(5,20); tft.println("Sensors warming up...");
          tft.setCursor(5,35); tft.print("Time rem: "); tft.print(remSec); tft.println(" s");
          // Serial debug
          Serial.printf("[ESP] Warming: %u s remaining\n", remSec);
        }
        break;  // stay in READ_ENV
      }
      if(!warmingDone){
        warmingDone = true;
        tft.fillScreen(BLACK);
      }
      readEnvSensors();
      state = CHECK_ALERTS;
      break;
    }

    case CHECK_ALERTS:
      checkAlerts();
      state = UPDATE_DISPLAY;
      break;

    case UPDATE_DISPLAY:
      updateDisplay();
      stateTs = now;
      state = IDLE;
      break;

    case IDLE:
      if(now - stateTs >= 2000){
        state = READ_NANO;  // loop back, no separate SEND_READY needed
      }
      break;
  }
}

void readFromNano(){
  while(nanoSerial.available()){
    String line = nanoSerial.readStringUntil('\n');
    line.trim();
    if(line.startsWith("MQ2:")){
      sensor_mq2 = line.substring(4).toInt();
      mq2 = sensor_mq2; // Update the global variable used in API
    }
    else if(line.startsWith("MQ135:")){
      sensor_mq135 = line.substring(6).toInt();
      mq135 = sensor_mq135; // Update the global variable used in API
    }
    else if(line.startsWith("Lux:")){
      sensor_lux = line.substring(4).toFloat();
      lux = sensor_lux; // Update the global variable used in API
    }
    else if(line.startsWith("DB:")){
      sensor_db = line.substring(3).toFloat();
      db = sensor_db; // Update the global variable used in API
    }
    else if(line.startsWith("VibX:")) {
      vibX = line.substring(5).toFloat();
    }
    else if(line.startsWith("VibY:")) {
      vibY = line.substring(5).toFloat();
    }
    else if(line.startsWith("VibZ:")) {
      vibZ = line.substring(5).toFloat();
    }
    else if(line.startsWith("VibMag:")) {
      vibMag = line.substring(7).toFloat();
    }
    else if(line == "---"){
      dataReceived = true;
    }
  }
}

void readGPS(){
  while(gpsSerial.available()){
    if(gps.encode(gpsSerial.read())){
      if(gps.time.isValid()){
        gpsHour=gps.time.hour();
        gpsMinute=gps.time.minute();
        gpsSecond=gps.time.second();
      }
      if(gps.date.isValid()){
        gpsDay=gps.date.day();
        gpsMonth=gps.date.month();
        gpsYear=gps.date.year();
      }
      if(gps.location.isValid()){
        latitude=gps.location.lat();
        longitude=gps.location.lng();
        gpsValid=true;
      }
      if(gps.satellites.isValid()){
        satellites=gps.satellites.value();
      }
    }
  }
}

void readEnvSensors(){
  if (bme.performReading()){
    temperature   = round(bme.temperature);
    pressure      = round(bme.pressure  / 100.0);
    humidity      = round(bme.humidity);
    gasResistance = round(bme.gas_resistance / 1000.0);
    
    // Calculate barometric altitude
    const float P0 = 1013.25; // Standard pressure at sea level in hPa
    float baroAltitude = 44330.0 * (1.0 - pow(pressure / P0, 0.1903));
    altitude = altitude * (1 + 0.00366 * temperature);
    
    // Get GPS altitude if valid
    float gpsAltitude = 0.0f;
    bool gpsAltitudeValid = false;
    if (gps.altitude.isValid()) {
      gpsAltitude = gps.altitude.meters();
      gpsAltitudeValid = true;
    }
    
    // Fuse the altitude measurements
    if (!gpsAltitudeValid) {
      altitude = baroAltitude;
    } else {
      // Complementary filter (98% baro, 2% GPS)
      altitude = 0.98 * baroAltitude + 0.02 * gpsAltitude;
    }
    
    // Berechne AirQ:
    float hum_score = (humidity >= 38 && humidity <= 42)
      ? 25.0f
      : (humidity < 38
         ? (humidity / 38.0f) * 25.0f
         : ((100.0f - humidity) / 58.0f) * 25.0f );
    float gas_score = (gasResistance >= 50)
      ? 75.0f
      : (gasResistance / 50.0f) * 75.0f;
    airQualityScore = hum_score + gas_score;
  }
  ccs.setEnvironmentalData(humidity, temperature);
  
  if(ccs.available() && !ccs.readData()){
    eco2 = ccs.geteCO2();
    tvoc = ccs.getTVOC();
  }
}


void checkAlerts(){
  alertState = false;
  alarmReason = "";
  
  // dynamic MQ thresholds
  static int base_mq2 = sensor_mq2, base_mq135 = sensor_mq135;
  base_mq2   = base_mq2*9/10 + sensor_mq2/10;
  base_mq135 = base_mq135*9/10 + sensor_mq135/10;
  
  if(sensor_mq2 > base_mq2*1.2f) {
    alertState = true;
    alarmReason += "MQ2 Gas, ";
  }
  
  if(sensor_mq135 > base_mq135*1.2f) {
    alertState = true;
    alarmReason += "MQ135 Air Quality, ";
  }
  
  // BME680 IAQ score
  float hum_score = (humidity>=38 && humidity<=42)
    ? 25.0f
    : (humidity<38 ? (humidity/38.0f)*25.0f : ((100.0f-humidity)/58.0f)*25.0f);
  float gas_score = gasResistance>=50
    ? 75.0f
    : (gasResistance/50.0f)*75.0f;
    
  if(hum_score+gas_score < 50.0f) {
    alertState = true;
    alarmReason += "BME680 Air Quality, ";
  }
  
  // CCS811
  if(eco2 > CCS811_CO2_THRESHOLD) {
    alertState = true;
    alarmReason += "CO2 Level, ";
  }
  
  if(tvoc > CCS811_TVOC_THRESHOLD) {
    alertState = true;
    alarmReason += "TVOC Level, ";
  }

  // Remove trailing comma if there's an alarm reason
  if (alarmReason.length() > 0) {
    alarmReason = alarmReason.substring(0, alarmReason.length() - 2);
  }

  if(alertState){
    tft.fillScreen(RED);
    tft.setTextSize(2); tft.setTextColor(WHITE);
    tft.setCursor(20,30); tft.println("ALERT!");
    
    // Display the alarm reason
    tft.setTextSize(1);
    tft.setCursor(10, 50);
    tft.println("Triggered by:");
    tft.setCursor(10, 60);
    tft.println(alarmReason);
  }
}

void updateDisplay(){
  tft.fillScreen(BLACK);
  tft.setTextSize(1); tft.setTextColor(CYAN);
  tft.setCursor(2,0); tft.print("Air Quality Monitor");
  // timestamp
  uint8_t dh = (gpsHour+SUMMER_TIME_OFFSET)%24;
  char buf[32];
  snprintf(buf,sizeof(buf),
    "%02u.%02u.%04u %02u:%02u:%02u",
    gpsDay,gpsMonth,gpsYear,dh,gpsMinute,gpsSecond);
  tft.setTextColor(WHITE); tft.setCursor(2,8); tft.print(buf);

  uint16_t y = HEADER_HEIGHT;
  tft.setCursor(2,y); tft.print("MQ-135: "); tft.println(sensor_mq135);y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("MQ-2:   "); tft.println(sensor_mq2);y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("dB:     "); tft.println(sensor_db,1);y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("Lux:    "); tft.println(sensor_lux,1);y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("Temp:   "); tft.print(temperature); tft.println(" C");y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("Hum:    "); tft.print(humidity);    tft.println(" %");y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("Pres:   "); tft.print(pressure);    tft.println(" hPa");y+=LINE_HEIGHT;
  tft.setCursor(2, y); tft.print("AirQ:  "); tft.print(airQualityScore, 1);y+= LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("eCO2:   "); tft.print(eco2);         tft.println(" ppm");y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("TVOC:   "); tft.print(tvoc);         tft.println(" ppb");y+=LINE_HEIGHT;
  tft.setCursor(2,y); tft.print("Vib:    "); tft.print(vibMag, 1); tft.println(" m/s2"); y+=LINE_HEIGHT;
  if(gpsValid){
    tft.setCursor(2,y); tft.print("Lat:    "); tft.println(latitude,6);  y+=LINE_HEIGHT;
    tft.setCursor(2,y); tft.print("Lon:    "); tft.println(longitude,6); y+=LINE_HEIGHT;
  } else {
    tft.setCursor(2,y); tft.println("GPS:    Searching...");        y+=LINE_HEIGHT;
  }
  tft.setCursor(2,y); tft.print("Sats:   "); tft.print(satellites);
  
}

void setupWebServer() {
  // Setup WiFi configuration page
  wifiManager.setupServer(&server);
  
  // Serve the main dashboard page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    String html = FPSTR(HTML_HEADER);
    html += FPSTR(DASHBOARD_CONTENT);
    html += "</body></html>";
    request->send(200, "text/html", html);
  });
  
  // Update the API endpoint to ensure airQualityScore is included
server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  
  StaticJsonDocument<512> doc;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  doc["pressure"] = pressure;
  doc["altitude"] = altitude;
  doc["eco2"] = eco2;
  doc["tvoc"] = tvoc;
  doc["mq2"] = sensor_mq2;  // Use sensor_mq2 instead of mq2
  doc["mq135"] = sensor_mq135;  // Use sensor_mq135 instead of mq135
  doc["lux"] = sensor_lux;  // Use sensor_lux instead of lux
  doc["db"] = sensor_db;  // Use sensor_db instead of db
  doc["vibMag"] = vibMag;
  doc["latitude"] = latitude;
  doc["longitude"] = longitude;
  doc["satellites"] = satellites;
  doc["airQualityScore"] = airQualityScore;
  doc["alertState"] = alertState;
  doc["alarmReason"] = alarmReason;
  
  serializeJson(doc, *response);
  request->send(response);
});
  
  // Start server
  server.begin();
  Serial.println("Web server started");
}