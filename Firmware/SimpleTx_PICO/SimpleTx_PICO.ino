/*
 * ELRS Pico Transmitter - Dual Mode Firmware
 * 
 * Mode is selected in config.h via BRIDGE_MODE define.
 * - BRIDGE_MODE 1: USB CRSF Bridge (PC -> USB -> UART -> TX)
 * - BRIDGE_MODE 0: Analog Gimbal Mode (Sticks -> UART -> TX)
 */

#include <Arduino.h>
#include <EEPROM.h>
#include "config.h"
#include "crsf.h"

// — Shared Globals —
CRSF crsfClass;
uint32_t currentMillis = 0;

// — Helper: Beep / LED —
void beep(int freq, int duration) {
    if (BUZZER_PIN > 0) tone(BUZZER_PIN, freq, duration);
}

void blink(int ms_on, int ms_off) {
    static uint32_t lastBlink = 0;
    static bool ledState = false;
    uint32_t period = ledState ? ms_on : ms_off;
    if (millis() - lastBlink > period) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastBlink = millis();
    }
}


// ======================================================================================
// PIPELINE 1: TRUE CRSF BRIDGE (BRIDGE_MODE = 1)
// ======================================================================================
#if BRIDGE_MODE

CRSFParser usbParser;
uint32_t lastRcFrameTime = 0;
uint32_t lastFailsafeTime = 0;
bool isFailsafeActive = false;

// Ring buffer for reverse bridge echo protection
#define ECHO_CACHE_SIZE 8
uint8_t sentCrcs[ECHO_CACHE_SIZE];
uint8_t sentCrcIdx = 0;

void recordSentFrameCrc(uint8_t crc) {
    sentCrcs[sentCrcIdx++] = crc;
    if (sentCrcIdx >= ECHO_CACHE_SIZE) sentCrcIdx = 0;
}

bool isEcho(uint8_t crc) {
    for (int i=0; i<ECHO_CACHE_SIZE; i++) {
        if (sentCrcs[i] == crc) return true;
    }
    return false;
}

void setupPipeline() {
    lastRcFrameTime = millis();
}

void sendFailsafe() {
    uint16_t fsChannels[16];
    for(int i=0; i<16; i++) fsChannels[i] = CRSF_CHANNEL_MID;
    
    // Safety
    fsChannels[THROTTLE] = CRSF_CHANNEL_MIN; 
    fsChannels[AUX1]     = CRSF_CHANNEL_MIN; // Disarm
    
    uint8_t fsFrame[CRSF_PACKET_SIZE];
    crsfBuildChannelsFrame(fsFrame, fsChannels);
    Serial1.write(fsFrame, CRSF_PACKET_SIZE);
}

void loopPipeline() {
    // 1. Forward Bridge (USB -> UART)
    while (Serial.available()) {
        uint8_t b = Serial.read();
        
        if (usbParser.feed(b)) {
            // Valid Frame Logic
            // NEW API: getFrame() returns buffer that persists until next valid frame (Double Buffered)
            const uint8_t* frame = usbParser.getFrame();
            uint8_t len = usbParser.getFrameLenTotal();
            
            // Forward
            Serial1.write(frame, len);
            
            // Record CRC for Echo Guard
            recordSentFrameCrc(frame[len-1]);

            // Stats / Failsafe Reset
            if (usbParser.getFrameType() == CRSF_TYPE_CHANNELS) {
                lastRcFrameTime = currentMillis;
                isFailsafeActive = false;
            }
            
            // Activity Toggle
            digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Fast blink
        }
    }

    // 2. Failsafe Injection
    if (currentMillis - lastRcFrameTime > FAILSAFE_TIMEOUT_MS) {
        if (!isFailsafeActive) {
            isFailsafeActive = true;
            #if DEBUG_LOG
             // Only safe to print if not using reverse bridge binary
            #endif
        }
        
        if (currentMillis - lastFailsafeTime > FAILSAFE_REPEAT_MS) {
            sendFailsafe();
            lastFailsafeTime = currentMillis;
            blink(100, 100); 
        }
    } else {
        // Normal blink by data toggle
    }

    // 3. Reverse Bridge (UART -> USB)
    #if ENABLE_REVERSE_BRIDGE
    static CRSFParser uartParser; 
    while (Serial1.available()) {
        uint8_t b = Serial1.read();
        if (uartParser.feed(b)) {
             // Valid frame from UART
             // Guard 1: Do not send Channels Back (Echo safety + Useless for PC usually)
             // Guard 2: Signature check
             
             if (uartParser.getFrameType() == CRSF_TYPE_CHANNELS) {
                 // Drop 0x16 to prevent control loops
                 continue; 
             }
             
             uint8_t crc = uartParser.getFrame()[uartParser.getFrameLenTotal()-1];
             if (!isEcho(crc)) {
                 Serial.write(uartParser.getFrame(), uartParser.getFrameLenTotal());
             }
        }
    }
    #endif
}

// ======================================================================================
// PIPELINE 2: ANALOG GIMBAL / POTENTIOMETER MODE (BRIDGE_MODE = 0)
// ======================================================================================
#else 

CalibValues cal;

long map_calibrated(long x, long in_min, long in_max, long in_center, long out_min, long out_max) {
    long out_center = (out_min + out_max) / 2;
    if (x < in_center) return map(x, in_min, in_center, out_min, out_center);
    else return map(x, in_center, in_max, out_center, out_max);
}

// Helper: Switch Logic (INPUT_PULLUP)
// LOW = Pressed (Active) = MAX VALUE (High Logic for Channel)
// HIGH = Open (Inactive) = MIN VALUE
uint16_t readSwitch(int pin) {
    return (digitalRead(pin) == LOW) ? CRSF_CHANNEL_MAX : CRSF_CHANNEL_MIN;
}

uint16_t read3PosSwitch(int pinHigh, int pinLow) {
    if (digitalRead(pinHigh) == LOW) return CRSF_CHANNEL_MAX; // Up
    if (digitalRead(pinLow) == LOW)  return CRSF_CHANNEL_MIN; // Down
    return CRSF_CHANNEL_MID; // Middle
}

void loadCalibration() {
    EEPROM.begin(512);
    if (EEPROM.read(CALIB_MARK_ADDR) == CALIB_MARK) {
        EEPROM.get(CALIB_VAL_ADDR, cal);
    } else {
        cal.aileronMin = CALIB_DEFAULT_MIN; cal.aileronMax = CALIB_DEFAULT_MAX; cal.aileronCenter = CALIB_DEFAULT_CENTER;
        cal.elevatorMin = CALIB_DEFAULT_MIN; cal.elevatorMax = CALIB_DEFAULT_MAX; cal.elevatorCenter = CALIB_DEFAULT_CENTER;
        cal.thrMin = CALIB_DEFAULT_MIN;     cal.thrMax = CALIB_DEFAULT_MAX;
        cal.rudderMin = CALIB_DEFAULT_MIN;  cal.rudderMax = CALIB_DEFAULT_MAX;  cal.rudderCenter = CALIB_DEFAULT_CENTER;
    }
}

void setupPipeline() {
    analogReadResolution(12); 
    
    pinMode(PIN_SWITCH_ARM, INPUT_PULLUP);
    pinMode(PIN_SWITCH_AUX2_HIGH, INPUT_PULLUP);
    pinMode(PIN_SWITCH_AUX2_LOW, INPUT_PULLUP);
    pinMode(PIN_SWITCH_AUX3, INPUT_PULLUP);
    pinMode(PIN_SWITCH_AUX4, INPUT_PULLUP);
    
    loadCalibration();
}

void loopPipeline() {
    static uint32_t lastSend = 0;
    
    if (micros() - lastSend > CRSF_SEND_RATE_US) {
        lastSend = micros();
        
        uint16_t crsfChannels[16];
        
        // 1. Read Analog
        int rawAil = analogRead(PIN_AILERON);
        int rawEle = analogRead(PIN_ELEVATOR);
        int rawThr = analogRead(PIN_THROTTLE);
        int rawRud = analogRead(PIN_RUDDER);
        
        // 2. Map
        crsfChannels[AILERON]  = constrain(map_calibrated(rawAil, cal.aileronMin, cal.aileronMax, cal.aileronCenter, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX), CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        crsfChannels[ELEVATOR] = constrain(map_calibrated(rawEle, cal.elevatorMin, cal.elevatorMax, cal.elevatorCenter, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX), CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        crsfChannels[THROTTLE] = constrain(map(rawThr, cal.thrMin, cal.thrMax, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX), CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        crsfChannels[RUDDER]   = constrain(map_calibrated(rawRud, cal.rudderMin, cal.rudderMax, cal.rudderCenter, CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX), CRSF_CHANNEL_MIN, CRSF_CHANNEL_MAX);
        
        // 3. Switches (Corrected Logic)
        crsfChannels[AUX1] = readSwitch(PIN_SWITCH_ARM); 
        crsfChannels[AUX2] = read3PosSwitch(PIN_SWITCH_AUX2_HIGH, PIN_SWITCH_AUX2_LOW);
        crsfChannels[AUX3] = readSwitch(PIN_SWITCH_AUX3);
        crsfChannels[AUX4] = readSwitch(PIN_SWITCH_AUX4);
        
        for (int i=8; i<16; i++) crsfChannels[i] = CRSF_CHANNEL_MID;
        
        uint8_t pkt[CRSF_PACKET_SIZE];
        crsfBuildChannelsFrame(pkt, crsfChannels);
        Serial1.write(pkt, CRSF_PACKET_SIZE);
        
        blink(200, 800);
    }
}

#endif // End Pipeline Split


// ======================================================================================
// MAIN SETUP
// ======================================================================================
void setup() {
    Serial.begin(115200);
    
    Serial1.setTX(CRSF_UART_TX_PIN);
    Serial1.setRX(CRSF_UART_RX_PIN);
    
    // Inversion Handling (Earle Philhower Core API)
    #if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
        // If config requires inversion, apply it.
        // Default is usually false.
        if (CRSF_UART_INVERT_TX) Serial1.setInvertTX(true);
        if (CRSF_UART_INVERT_RX) Serial1.setInvertRX(true);
    #endif
    
    Serial1.begin(CRSF_UART_BAUD);
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    setupPipeline();
    
    #if DEBUG_LOG
      Serial.println(BRIDGE_MODE ? "BRIDGE" : "ANALOG");
    #endif
}

void loop() {
    currentMillis = millis();
    loopPipeline();
}
