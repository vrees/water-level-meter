#include <Arduino.h>
#include <U8x8lib.h>
#include <Wire.h>

/*
* eTapeContinuous Fluid Level Sensor PN-12110215TC-24  PN-6573TC-24
* Sensor Length: 660mm
* Active Sensor Length:12.4" 660mm
* Reference Resistor (Rref): 3000Ω, ±10%    gemessen:   3200Ω
* Sensor Output empty:       3000Ω          
* Sensor Output: full:        300Ω          gemessen:   400Ω
* Resistance Gradient: 44/cm, ±10%          
* PowerRating:  0.5 Watts (VMax = 10V)
*
* Voltage Gradient bei 3.3V:                gemessen:   48.7 V/cm
* s [cm] = 48.7cm/V * U[V] - 17.3cm         gerechnet:  46,5 V/cm
*
*/
#define HAS_LORA 1         // comment out if device shall not send data via LoRa
#define CFG_sx1276_radio 1 // HPD13A LoRa SoC

#define HAS_LED NOT_A_PIN // on-board LED is wired to SCL (used by display) therefore totally useless

// disable brownout detection (needed on TTGOv2 for battery powered operation)
#define DISABLE_BROWNOUT 1 // comment out if you want to keep brownout feature

// Pins for LORA chip SPI interface come from board file, we need some
// additional definitions for LMIC
// #define LORA_RST  LMIC_UNUSED_PIN
#define LORA_IO1 (33)
#define LORA_IO2 LMIC_UNUSED_PIN

//For TTGO LoRa32 V2 use:
#define MY_OLED_SDA (21)
#define MY_OLED_SCL (22)
#define MY_OLED_RST (16)

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(MY_OLED_RST, MY_OLED_SCL, MY_OLED_SDA);

const byte VOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 12;    // 10 - 12 bits

float voltage = 0;
uint16_t adc_reading = 0;
uint16_t counter = 0;

void read_voltage()
{
  adc_reading = analogRead(VOLTAGE_PIN);
  // Convert ADC reading to voltage in deciVolt, 1024/2048/4096 not hardcoded but calculated depending on the set ADC_BITS

  voltage = adc_reading * 2.20 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096)
}

float calcHeigth()
{
  float height = 48.7 * voltage - 17.3;
  return -height;
}

double readVoltageCompensated(byte pin);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("ADC Measurements on the ESP32");

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_6db);  // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
  pinMode(VOLTAGE_PIN, INPUT);
}

void loop()
{
  char buff[16];

  read_voltage();

  u8x8.inverse();
  snprintf(buff, sizeof(buff), "U:%.2fV", voltage);
  u8x8.draw2x2String(0, 0, buff);

  u8x8.noInverse();
  // u8x8.setCursor(0, 3);
  // u8x8.printf("ADC:%-4d", adc_reading);

  snprintf(buff, sizeof(buff), "ADC:%-4d", adc_reading);
  u8x8.draw2x2String(0, 3, buff);

  float height = calcHeigth();
  snprintf(buff, sizeof(buff), "%.1fcm", height);
  u8x8.draw2x2String(0, 5, buff);

  counter++;
  u8x8.setCursor(10, 7);
  u8x8.printf("%i", counter);

  Serial.printf("U=%.2fV\t\tADC=%-4d\t\tHeight=:%.1fcm\n", voltage, adc_reading, height);

  delay(2000);
}