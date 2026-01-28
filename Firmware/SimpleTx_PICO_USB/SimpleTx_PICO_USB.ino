#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>

#include "config.h"
#include "led.h"
#include "tone.h"
#include "NonBlockingRtttl.h"
#include "crsf.h"

// — Globals
Adafruit_ADS1115 ads;                       // default 0x48 address (Used for Battery Monitoring)
CRSF crsfClass;
uint8_t crsfPacket[CRSF_PACKET_SIZE];
int16_t rcChannels[CRSF_MAX_CHANNEL];
float   batteryVoltage = 0;
uint32_t currentMillis = 0;

// — USB Serial Input Globals
int serialChannels[8]; // Stores T, A, E, R, Aux1, Aux2, Aux3, Aux4 (0-7)

// — Settings Globals
uint8_t currentPktRate = 0; // 0=50Hz, 1=100Hz, 2=250Hz? Need to verify ELRS constants
uint8_t currentPower = 0;   // 0=10mW, 1=25mW, ...
uint8_t currentSetting = 0;
uint8_t crsfCmdPacket[CRSF_CMD_PACKET_SIZE];
unsigned long lastSettingsChange = 0;

// — Inactivity Globals
unsigned long lastStickInput = 0;
#define INACTIVITY_TIMEOUT 600000 // 10 minutes (600,000 ms)

// Helper: Send ELRS Command
void sendElrsCommand(uint8_t command, uint8_t value) {
    crsfClass.crsfPrepareCmdPacket(crsfCmdPacket, command, value);
    crsfClass.CrsfWritePacket(crsfCmdPacket, CRSF_CMD_PACKET_SIZE);
}

// Helper: Feedback for settings change
void settingsFeedback(int count) {
    for (int i=0; i<count; i++) {
        digitalWrite(LED_PIN, HIGH);
        tone(BUZZER_PIN, 2000, 50);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

// Check Stick Commands (Elevator Up/Down + Aileron Left/Right)
void checkStickCommands() {
    // Debounce: Only allow change every 2 seconds
    if (millis() - lastSettingsChange < 2000) return;

    // Thresholds (Using standard logical ranges)
    bool elUp = (rcChannels[ELEVATOR] > 1800);
    bool elDown = (rcChannels[ELEVATOR] < 1200);
    bool ailLeft = (rcChannels[AILERON] < 1200);
    bool ailRight = (rcChannels[AILERON] > 1800);
    
    // Command 1: EL Un + Ail Left -> 50Hz / 25mW
    if (elUp && ailLeft) {
        sendElrsCommand(ELRS_PKT_RATE_COMMAND, 0); // 0 = 50Hz
        delay(100);
        sendElrsCommand(ELRS_POWER_COMMAND, 1);    // 1 = 25mW
        settingsFeedback(1);
        lastSettingsChange = millis();
    }
    // Command 2: EL Up + Ail Right -> 250Hz / 100mW
    else if (elUp && ailRight) {
        sendElrsCommand(ELRS_PKT_RATE_COMMAND, 3); // 3 = 250Hz
        delay(100);
        sendElrsCommand(ELRS_POWER_COMMAND, 3);    // 3 = 100mW
        settingsFeedback(2);
        lastSettingsChange = millis();
    }
    // Command 3: EL Down + Ail Left -> BIND
    else if (elDown && ailLeft) {
        sendElrsCommand(ELRS_BIND_COMMAND, 0);
        settingsFeedback(4); 
        lastSettingsChange = millis(); 
    }
    // Command 4: EL Down + Ail Right -> WiFi
    else if (elDown && ailRight) {
        sendElrsCommand(ELRS_WIFI_COMMAND, 0);
        settingsFeedback(3);
        lastSettingsChange = millis();
    }
}

// Helper: Parse CSV Input "T,A,E,R,Aux1,Aux2,Aux3,Aux4"
void parseSerialInput() {
    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
            // Very simple parser for 8 CSV values
            int start = 0;
            int idx = 0;
            for (int i = 0; i < line.length(); i++) {
                if (line.charAt(i) == ',' || i == line.length() - 1) {
                    if (i == line.length() - 1 && line.charAt(i) != ',') i++; // include last char
                    
                    String valStr = line.substring(start, i);
                    if (idx < 8) {
                        serialChannels[idx] = valStr.toInt();
                        // Constrain safety
                        serialChannels[idx] = constrain(serialChannels[idx], 800, 2200); 
                    }
                    start = i + 1;
                    idx++;
                }
            }
            
            // Check Inactivity (Reset timer if we got a packet)
            lastStickInput = millis();
        }
    }
}

void setup() {
  Wire.begin();
  // ADS1115 for Battery
  if (!ads.begin(ADC_I2C_ADDRESS)) {
    // Fail silently or blink? Just continue, maybe no battery monitor desired.
  }
  ads.setGain(ADC_GAIN);

  Serial.begin(115200);      // USB Serial for Control Input
  Serial1.begin(SERIAL_BAUDRATE); // CRSF to Module
  crsfClass.begin();

  pinMode(LED_PIN,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Defaults
  for(int i=0; i<8; i++) serialChannels[i] = 1500;
  serialChannels[2] = 1000; // Throttle min default
  serialChannels[4] = 1000; // Aux1 (Arm) Disarmed default
  
  // Quick startup beep
  digitalWrite(LED_PIN, HIGH);
  tone(BUZZER_PIN, 2000, 100);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  currentMillis = millis();

  // 1) Read USB Serial Input
  parseSerialInput();

  // 2) Map Serial Values to CRSF Channels
  // Order expected: Thr, Ail, Ele, Rud, Aux1, Aux2, Aux3, Aux4
  // CRSF Mapping: Aileron, Elevator, Throttle, Rudder
  rcChannels[AILERON]  = serialChannels[1];
  rcChannels[ELEVATOR] = serialChannels[2];
  rcChannels[THROTTLE] = serialChannels[0];
  rcChannels[RUDDER]   = serialChannels[3];
  rcChannels[AUX1]     = serialChannels[4];
  rcChannels[AUX2]     = serialChannels[5];
  rcChannels[AUX3]     = serialChannels[6];
  rcChannels[AUX4]     = serialChannels[7];

  // 3) Inactivity Alarm
  if (currentMillis - lastStickInput > INACTIVITY_TIMEOUT) {
      if ((currentMillis / 500) % 2) {
          tone(BUZZER_PIN, 2000, 100);
          digitalWrite(LED_PIN, HIGH);
      } else {
          digitalWrite(LED_PIN, LOW);
      }
  }

  // 4) Battery Monitor
  int16_t rawBat = ads.readADC_SingleEnded(ADC_CHANNEL_BATTERY);
  batteryVoltage = rawBat * ADC_LSB_SIZE * VOLTAGE_DIVIDER;
  if (batteryVoltage < BATTERY_VOLT_MIN && currentMillis % 1000 < 100) { // Beep once per second
    tone(BUZZER_PIN, 1000, 100);
  }
  
  // 5) Send CRSF
  checkStickCommands();
  
  crsfClass.crsfPrepareDataPacket(crsfPacket, rcChannels);
  crsfClass.CrsfWritePacket(crsfPacket, CRSF_PACKET_SIZE);
  
  // Feedback
  digitalWrite(LED_PIN, (currentMillis & 0x200) ? HIGH : LOW);
  playingTones(1);
}
