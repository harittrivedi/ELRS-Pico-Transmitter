#ifndef CONFIG_PICO_ADC_H
#define CONFIG_PICO_ADC_H

#include <Adafruit_ADS1X15.h>

// — ADS1115 setup
#define ADC_I2C_ADDRESS     0x48
#define ADC_CHANNEL_BATTERY 0       // AIN0
#define ADC_GAIN            GAIN_ONE
#define ADC_LSB_SIZE        (4.096f / 32767.0f)
#define VOLTAGE_DIVIDER     2.0f    // 100k/100k

// — Battery thresholds
#define BATTERY_VOLT_MIN  3.0f
#define BATTERY_VOLT_MAX  8.4f

// — CRSF serial baud
#define SERIAL_BAUDRATE   400000

// — ADC resolution
#define ADC_MIN  0
#define ADC_MAX  4095

// — Sticker Analog Pins (Standard Joystick Pots are 10k or 5k)
#define ANALOG_CUTOFF 20 // Range padding

// — Stick ADC pins
const int PIN_AILERON  = A3;  // GP29 / ADC3
const int PIN_ELEVATOR = A2;  // GP28 / ADC2
const int PIN_THROTTLE = A1;  // GP27 / ADC1
const int PIN_RUDDER   = A0;  // GP26 / ADC0

// — Switch pins (use 10 kΩ externals for reliability)
const int PIN_SWITCH_ARM       = 2;
const int PIN_SWITCH_AUX2_HIGH = 3;
const int PIN_SWITCH_AUX2_LOW  = 6;
const int PIN_SWITCH_AUX3      = 7;
const int PIN_SWITCH_AUX4      = 8;

// — LED & buzzer (renamed to avoid core macro conflicts)
const int LED_PIN    = 9;
const int BUZZER_PIN = 10;

// — RC command limits (from your config.h)
#define RC_MIN_COMMAND 600
#define RC_MAX_COMMAND 1400

// — Channel order enum
enum chan_order {
  AILERON,
  ELEVATOR,
  THROTTLE,
  RUDDER,
  AUX1,
  AUX2,
  AUX3,
  AUX4
};

// — Calibration Structure & Storage
#define CALIB_MARK      0x55    // Magic byte
#define CALIB_MARK_ADDR 0       // Address for magic byte
#define CALIB_VAL_ADDR  1       // Address for calibration struct

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
