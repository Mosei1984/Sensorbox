#include <Arduino.h>
#include <SoftwareSerial.h>
#include <math.h>

// —— Pin‐Belegung —— 
#define MQ2_PIN     A0
#define MQ135_PIN   A1
#define LDR_PIN     A2
#define MIC_PIN     A4

// —— Parameter —— 
const uint16_t NUM_SAMPLES       = 100;   // für MQ & LDR  
const uint16_t MIC_WINDOW_MS     = 50;    // Mikro-Fenster  
const unsigned long HANDSHAKE_TO = 1000;  // ms  
const unsigned long SEND_INTERVAL=1000;  // ms  

// —— LDR→Lux —— 
const float LDR_R_FIXED = 10000.0f;
const float LUX_A       = 500.0f;
const float LUX_B       = -1.9f;

// —— SoftwareSerial zum ESP32 (TX→Pin10) —— 
SoftwareSerial espSerial(9, 10);

// —— State‐Machine —— 
enum State {
  WAIT_HANDSHAKE,
  SAMPLING_MQ_LDR,
  SAMPLING_MIC,
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

void setup() {
  Serial.begin(115200);
  espSerial.begin(9600);
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
          samplesLeft = NUM_SAMPLES;
          sum2 = sum135 = sumLdr = 0;
          state = SAMPLING_MQ_LDR;
          stateTs = 0;
        }
      } else if (now - stateTs >= HANDSHAKE_TO) {
        // Timeout: restart waiting
        stateTs = now;
      }
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
        state = SEND;
      }
      break;

    // ——— 4) Werte senden ————————————————————————
    case SEND:
      // USB‐Debug
      Serial.print("MQ2: ");    Serial.print(mq2Val,   1);
      Serial.print("\tMQ135: ");Serial.print(mq135Val, 1);
      Serial.print("\tLux: ");  Serial.print(luxVal,    1); Serial.print(" lx");
      Serial.print("\tdB: ");   Serial.print(dbVal,     1); Serial.println(" dB");
      // an ESP32
      espSerial.print("MQ2:");   espSerial.println(mq2Val,   1);
      espSerial.print("MQ135:"); espSerial.println(mq135Val, 1);
      espSerial.print("Lux:");   espSerial.println(luxVal,   1);
      espSerial.print("DB:");    espSerial.println(dbVal,    1);
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