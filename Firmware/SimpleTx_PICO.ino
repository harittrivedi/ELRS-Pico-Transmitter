#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#include "config.h"
#include "led.h"
#include "tone.h"
#include "NonBlockingRtttl.h"
#include "crsf.h"

// — Globals
Adafruit_ADS1115 ads;                       // default 0x48 address
CRSF crsfClass;
uint8_t crsfPacket[CRSF_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
float   batteryVoltage = 0;
uint32_t currentMillis = 0;

void setup() {
  // I2C (RP2040 core only supports no-arg begin)
  Wire.begin();

  // ADS1115 init
  if (!ads.begin(ADC_I2C_ADDRESS)) {
    Serial.begin(115200);
    while (1) Serial.println("✖ ADS1115 not found");
  }
  ads.setGain(ADC_GAIN);

  // Serial ports
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial1.begin(SERIAL_BAUDRATE);
  crsfClass.begin();

  // I/O modes
  pinMode(LED_PIN,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PIN_SWITCH_ARM,       INPUT_PULLUP);
  pinMode(PIN_SWITCH_AUX2_HIGH, INPUT_PULLUP);
  pinMode(PIN_SWITCH_AUX2_LOW,  INPUT_PULLUP);
  pinMode(PIN_SWITCH_AUX3,      INPUT_PULLUP);
  pinMode(PIN_SWITCH_AUX4,      INPUT_PULLUP);
}

void loop() {
  currentMillis = millis();

  // 1) Read sticks → map into [RC_MIN_COMMAND,RC_MAX_COMMAND]
  rcChannels[AILERON]  = map(analogRead(PIN_AILERON),  0, ADC_MAX, RC_MIN_COMMAND, RC_MAX_COMMAND);
  rcChannels[ELEVATOR] = map(analogRead(PIN_ELEVATOR), 0, ADC_MAX, RC_MIN_COMMAND, RC_MAX_COMMAND);
  rcChannels[THROTTLE] = map(analogRead(PIN_THROTTLE), 0, ADC_MAX, RC_MIN_COMMAND, RC_MAX_COMMAND);
  rcChannels[RUDDER]   = map(analogRead(PIN_RUDDER),   0, ADC_MAX, RC_MIN_COMMAND, RC_MAX_COMMAND);

  // 2) Battery via ADS1115
  int16_t raw = ads.readADC_SingleEnded(ADC_CHANNEL_BATTERY);
  batteryVoltage = raw * ADC_LSB_SIZE * VOLTAGE_DIVIDER;

  // 3) Under/Over-voltage alerts
  if (batteryVoltage < BATTERY_VOLT_MIN) {
    tone(BUZZER_PIN, 1000, 200);
    Serial.println("⚠ UNDERVOLTAGE");
  }
  else if (batteryVoltage > BATTERY_VOLT_MAX) {
    Serial.println("⚠ OVERVOLTAGE");
  }

  // 4) Read switches
  rcChannels[AUX1] = digitalRead(PIN_SWITCH_ARM)       ? RC_MIN_COMMAND : RC_MAX_COMMAND;
  if (digitalRead(PIN_SWITCH_AUX2_HIGH))  rcChannels[AUX2] = RC_MAX_COMMAND;
  else if (digitalRead(PIN_SWITCH_AUX2_LOW)) rcChannels[AUX2] = RC_MIN_COMMAND;
  rcChannels[AUX3] = digitalRead(PIN_SWITCH_AUX3)     ? RC_MIN_COMMAND : RC_MAX_COMMAND;
  rcChannels[AUX4] = digitalRead(PIN_SWITCH_AUX4)     ? RC_MIN_COMMAND : RC_MAX_COMMAND;

  // 5) Build & send CRSF packet
  crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
  crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);

  // 6) Feedback
  digitalWrite(LED_PIN, (currentMillis & 0x200) ? HIGH : LOW);
  playingTones(1);   // satisfy playingTones(uint8_t) signature
}
