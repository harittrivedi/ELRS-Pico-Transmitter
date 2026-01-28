#ifndef CONFIG_PICO_ADC_H
#define CONFIG_PICO_ADC_H

#include <Arduino.h> 

// — OPERATION MODE —
#define BRIDGE_MODE           1   // 1 = True CRSF Bridge (USB->UART), 0 = Analog Gimbal/Pot Mode

#define DEBUG_LOG             0   // 1 = Enable Log

// — CRSF UART CONFIG —
#define CRSF_UART_TX_PIN      0
#define CRSF_UART_RX_PIN      1
#define CRSF_UART_BAUD        400000
#define CRSF_UART_INVERT_TX   false 
#define CRSF_UART_INVERT_RX   false
#define ENABLE_REVERSE_BRIDGE 1     

// — FAILSAFE / SAFETY —
#define FAILSAFE_TIMEOUT_MS   250   
#define FAILSAFE_REPEAT_MS    20    
#define CRSF_CHANNEL_MIN      172
#define CRSF_CHANNEL_MID      992
#define CRSF_CHANNEL_MAX      1811

// ==========================================
// MODE 0: ANALOG GIMBAL CONFIGURATION
// ==========================================
#if BRIDGE_MODE == 0

    // — Stick ADC pins (Common Pico Assignments) —
    const int PIN_AILERON  = 29;  // A3 / GP29 
    const int PIN_ELEVATOR = 28;  // A2 / GP28 
    const int PIN_THROTTLE = 27;  // A1 / GP27 
    const int PIN_RUDDER   = 26;  // A0 / GP26 
    
    // — Switch Pins (Digital Inputs) —
    const int PIN_SWITCH_ARM       = 2;  // Aux1 (Arm)
    const int PIN_SWITCH_AUX2_HIGH = 3;  // Aux2
    const int PIN_SWITCH_AUX2_LOW  = 6;
    const int PIN_SWITCH_AUX3      = 7;  // Aux3
    const int PIN_SWITCH_AUX4      = 8;  // Aux4

    // — ADC Settings —
    #define ANALOG_SMOOTHING 0.5f // Simple IIR filter alpha (optional)
    #define ADC_MAX_VAL      4095
    #define CRSF_SEND_RATE_US 5000 // 200Hz

    // — Default Calibration (if EEPROM empty) —
    // Assumes 12-bit ADC (0-4095). Center approx 2048.
    #define CALIB_DEFAULT_MIN    100
    #define CALIB_DEFAULT_MAX    3995
    #define CALIB_DEFAULT_CENTER 2048

#endif

// — LED / BUZZER —
// — LED / BUZZER —
#ifdef LED_BUILTIN
  #define LED_PIN             LED_BUILTIN
#else
  #define LED_PIN             25 // Default for Pico if macro missing
#endif
#define BUZZER_PIN            10

// — Channel Map —
enum chan_order {
  AILERON = 0,
  ELEVATOR,
  THROTTLE,
  RUDDER,
  AUX1, // Arm
  AUX2,
  AUX3,
  AUX4
  // ...
};

// — Calibration Structure —
#define CALIB_MARK      0x55    
#define CALIB_MARK_ADDR 0       
#define CALIB_VAL_ADDR  1       

struct CalibValues {
    int aileronMin;
    int aileronMax;
    int aileronCenter;
    int elevatorMin;
    int elevatorMax;
    int elevatorCenter;
    int thrMin;
    int thrMax;
    int rudderMin;
    int rudderMax;
    int rudderCenter;
};

#endif // CONFIG_PICO_ADC_H
