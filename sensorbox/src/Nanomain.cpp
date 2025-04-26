#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>

// —— Pin‐Belegung —— 
#define MQ2_PIN     A0
#define MQ135_PIN   A1
#define LDR_PIN     A2
#define MIC_PIN     A6
#ifndef A0
  #define A0 14  // Analog pin 0 on Arduino Nano
  #define A1 15  // Analog pin 1 on Arduino Nano
  #define A2 16  // Analog pin 2 on Arduino Nano
#endif
// —— ADXL345 Definitions ——
#define ADXL345_ADDR 0x53  // I2C address of ADXL345
#define POWER_CTL    0x2D  // Power Control Register
#define DATA_FORMAT  0x31  // Data Format Register
#define DATAX0       0x32  // X-Axis Data 0 Register
#define DATAY0       0x34  // Y-Axis Data 0 Register
#define DATAZ0       0x36  // Z-Axis Data 0 Register

// —— Parameter —— 
const uint16_t NUM_SAMPLES       = 100;   // für MQ & LDR  
const uint16_t MIC_WINDOW_MS     = 50;    // Mikro-Fenster  
const unsigned long HANDSHAKE_TO = 1000;  // ms  
const unsigned long SEND_INTERVAL= 1000;  // ms  
const uint16_t ACCEL_CAL_SAMPLES = 50;    // Anzahl der Kalibrierungsproben

// —— LDR→Lux —— 
const float LDR_R_FIXED = 10000.0f;
const float LUX_A       = 500.0f;
const float LUX_B       = -1.8f;

// —— SoftwareSerial zum ESP32 (TX→Pin10) —— 
SoftwareSerial espSerial(9, 10);

// —— Accelerometer variables —— 
float vibX, vibY, vibZ, vibMag;
float offsetX, offsetY, offsetZ;  // Kalibrierungsoffsets
bool isCalibrated = false;        // Kalibrierungsstatus

// —— State‐Machine —— 
enum State {
  WAIT_HANDSHAKE,
  CALIBRATE_ACCEL,
  SAMPLING_MQ_LDR,
  SAMPLING_MIC,
  SAMPLING_ACCEL,
  SEND,
  IDLE
};
State state = WAIT_HANDSHAKE;

// —— FSM‐Variablen —— 
unsigned long stateTs = 0;
uint16_t     samplesLeft;
uint32_t     sum2, sum135, sumLdr;
uint16_t     micMax, micMin;
float        mq2Val, mq135Val, luxVal, dbVal;

// Function to write to ADXL345 register
void writeRegister(byte reg, byte value) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Function to read raw ADXL345 registers (ohne Kalibrierung)
void readRawAccel(float &x, float &y, float &z) {
  // Die Register DATAX0, DATAY0, DATAZ0 (0x32, 0x34, 0x36) folgen direkt aufeinander,
  // daher können wir alle 6 Bytes (X, Y, Z) in einer Transaktion lesen
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(DATAX0);  // Beginnt bei DATAX0 (0x32)
  Wire.endTransmission();
  
  Wire.requestFrom(ADXL345_ADDR, 6);  // Liest 6 Bytes: DATAX0, DATAX1, DATAY0, DATAY1, DATAZ0, DATAZ1
  
  int16_t rawX = Wire.read() | (Wire.read() << 8);  // DATAX0, DATAX1
  int16_t rawY = Wire.read() | (Wire.read() << 8);  // DATAY0, DATAY1
  int16_t rawZ = Wire.read() | (Wire.read() << 8);  // DATAZ0, DATAZ1
  
  // Convert to g (scale factor for ±16g range is 31.2 mg/LSB)
  x = rawX * 0.0312;
  y = rawY * 0.0312;
  z = rawZ * 0.0312;
}

// Function to read calibrated ADXL345 values (mit Kalibrierung und ohne Erdbeschleunigung)
void readAccel(float &x, float &y, float &z) {
  float rawX, rawY, rawZ;
  
  // Rohdaten lesen
  readRawAccel(rawX, rawY, rawZ);
  
  // Kalibrierungsoffsets anwenden (entfernt Bias und Erdbeschleunigung)
  x = rawX - offsetX;
  y = rawY - offsetY;
  z = rawZ - offsetZ;
}

// Kalibrierungsfunktion - misst die Ruhelage und berechnet Offsets
void calibrateAccelerometer() {
  float sumX = 0, sumY = 0, sumZ = 0;
  
  Serial.println("Kalibriere ADXL345...");
  
  // Mehrere Messungen durchführen und Durchschnitt berechnen
  for (uint16_t i = 0; i < ACCEL_CAL_SAMPLES; i++) {
    float x, y, z;
    readRawAccel(x, y, z);
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(10);  // Kurze Pause zwischen den Messungen
  }
  
  // Durchschnittliche Ruhelage berechnen (enthält Erdbeschleunigung)
  offsetX = sumX / ACCEL_CAL_SAMPLES;
  offsetY = sumY / ACCEL_CAL_SAMPLES;
  offsetZ = sumZ / ACCEL_CAL_SAMPLES;
  
  Serial.println("Kalibrierung abgeschlossen!");
  Serial.print("Offsets: X=");
  Serial.print(offsetX);
  Serial.print(" Y=");
  Serial.print(offsetY);
  Serial.print(" Z=");
  Serial.println(offsetZ);
  
  isCalibrated = true;
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
  
  // Initialize I2C
  Wire.begin();
  
  // Initialize ADXL345
  // Set to measurement mode
  writeRegister(POWER_CTL, 0x08);
  
  // Set data format to full resolution, 16g range
  writeRegister(DATA_FORMAT, 0x0B);
  
  delay(100); // Short delay to let the sensor initialize
  
  Serial.println("ADXL345 initialisiert. Bitte Gerät ruhig halten für Kalibrierung.");
}

float computeLux(uint32_t sum, uint16_t count) {
  float raw  = float(sum) / count;
  float Vout = raw * (5.0f / 1023.0f);
  float Rldr = LDR_R_FIXED * ( Vout / (5.0f - Vout) );
  return LUX_A * pow(Rldr / 1000.0f, LUX_B);
}

void loop() {
  unsigned long now = millis();

  switch (state) {
    // ——— 1) Auf Handshake warten —————————————————
    case WAIT_HANDSHAKE:
      if (stateTs == 0) stateTs = now;  // Start-Timeout
      if (espSerial.available()) {
        String line = espSerial.readStringUntil('\n');
        line.trim();
        if (line == "READY") {
          // Got it!
          if (!isCalibrated) {
            // Wenn noch nicht kalibriert, zuerst kalibrieren
            state = CALIBRATE_ACCEL;
          } else {
            // Sonst direkt mit Sampling beginnen
            samplesLeft = NUM_SAMPLES;
            sum2 = sum135 = sumLdr = 0;
            state = SAMPLING_MQ_LDR;
          }
          stateTs = 0;
        }
      } else if (now - stateTs >= HANDSHAKE_TO) {
        // Timeout: restart waiting
        stateTs = now;
      }
      break;
      
    // ——— 1.5) Accelerometer kalibrieren —————————————
    case CALIBRATE_ACCEL:
      calibrateAccelerometer();
      // Nach der Kalibrierung mit normalen Sampling fortfahren
      samplesLeft = NUM_SAMPLES;
      sum2 = sum135 = sumLdr = 0;
      state = SAMPLING_MQ_LDR;
      break;

    // ——— 2) MQ-2, MQ-135, LDR mitteln —————————————————
    case SAMPLING_MQ_LDR:
      if (samplesLeft > 0) {
        sum2   += analogRead(MQ2_PIN);
        sum135 += analogRead(MQ135_PIN);
        sumLdr += analogRead(LDR_PIN);
        samplesLeft--;
      } else {
        mq2Val   = float(sum2)   / NUM_SAMPLES;
        mq135Val = float(sum135) / NUM_SAMPLES;
        luxVal   = computeLux(sumLdr, NUM_SAMPLES);
        // Prepare mic sampling
        micMax = 0;
        micMin = 1023;
        state = SAMPLING_MIC;
        stateTs = now;
      }
      break;

    // ——— 3) Mikrofon Peak-to-Peak messen —————————————
    case SAMPLING_MIC:
      if (now - stateTs < MIC_WINDOW_MS) {
        uint16_t v = analogRead(MIC_PIN);
        if (v > micMax) micMax = v;
        if (v < micMin) micMin = v;
      } else {
        float Vpp = (micMax - micMin) * (5.0f / 1023.0f);
        dbVal = 20.0f * log10(Vpp / 0.005f) + 30.0f; // dB
        state = SAMPLING_ACCEL;
      }
      break;
      
    // ——— 3.5) ADXL345 Accelerometer messen —————————————
    case SAMPLING_ACCEL:
      {
        // Kalibrierte Beschleunigungsdaten lesen (ohne Erdbeschleunigung)
        readAccel(vibX, vibY, vibZ);
        
        // Berechne die Magnitude der Vibration (Vektorbetrag)
        vibMag = sqrt(vibX*vibX + vibY*vibY + vibZ*vibZ);
        
        state = SEND;
      }
      break;

    // ——— 4) Werte senden ————————————————————————
    case SEND:
      // USB‐Debug
      Serial.print("MQ2: ");    Serial.print(mq2Val,   1);
      Serial.print("\tMQ135: ");Serial.print(mq135Val, 1);
      Serial.print("\tLux: ");  Serial.print(luxVal,    1); Serial.print(" lx");
      Serial.print("\tdB: ");   Serial.print(dbVal,     1); Serial.print(" dB");
      Serial.print("\tVibX: "); Serial.print(vibX,      2);
      Serial.print("\tVibY: "); Serial.print(vibY,      2);
      Serial.print("\tVibZ: "); Serial.print(vibZ,      2);
      Serial.print("\tVib: ");  Serial.print(vibMag,    2); Serial.println(" g");
      
      // an ESP32
      espSerial.print("MQ2:");   espSerial.println(mq2Val,   1);
      espSerial.print("MQ135:"); espSerial.println(mq135Val, 1);
      espSerial.print("Lux:");   espSerial.println(luxVal,   1);
      espSerial.print("DB:");    espSerial.println(dbVal,    1);
      espSerial.print("VibX:");  espSerial.println(vibX,     2);
      espSerial.print("VibY:");  espSerial.println(vibY,     2);
      espSerial.print("VibZ:");  espSerial.println(vibZ,     2);
      espSerial.print("VibMag:");espSerial.println(vibMag,   2);
      espSerial.println("---");
      
      // next
      stateTs = now;
      state   = IDLE;
      break;

    // ——— 5) Intervall warten ohne Delay ———————————————
    case IDLE:
      if (now - stateTs >= SEND_INTERVAL) {
        state = WAIT_HANDSHAKE;
        stateTs = 0;
      }
      break;
  }
}