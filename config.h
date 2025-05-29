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
#define RC_MIN_COMMAND 600     // :contentReference[oaicite:5]{index=5}
#define RC_MAX_COMMAND 1400    // :contentReference[oaicite:7]{index=7}

// — Channel order enum (so AILERON, ELEVATOR, etc. resolve) :contentReference[oaicite:9]{index=9}
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

#endif // CONFIG_PICO_ADC_H
