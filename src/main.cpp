#include <Arduino.h>
#include <SPI.h>
#include <U8x8lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

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
#define MY_OLED_SDA (4)
#define MY_OLED_SCL (15)
#define MY_OLED_RST (16)

U8X8_SSD1306_128X64_NONAME_HW_I2C display(MY_OLED_RST, MY_OLED_SCL, MY_OLED_SDA);

const byte VOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 12;    // 10 - 12 bits

uint16_t read_voltage()
{
  uint32_t adc_reading = analogRead(VOLTAGE_PIN);
  
  Serial.print(F("ADC="));
  Serial.println(adc_reading);
  // Convert ADC reading to voltage in deciVolt, 1024/2048/4096 not hardcoded but calculated depending on the set ADC_BITS

  double voltage = adc_reading * 2.2 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096)

  // Serial.print(F(", Voltage="));
  // Serial.println(voltage);

  Serial.print(voltage);
  Serial.print(" V \t");
  Serial.println(adc_reading);

  return voltage;
}

double readVoltage(byte pin);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("ADC Measurements on the ESP32"));
  display.begin();
  display.refreshDisplay();
  display.clearDisplay();
  display.setFont(u8x8_font_chroma48medium8_r);

  analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_6db);  // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
  pinMode(VOLTAGE_PIN, INPUT);
}

void loop()
{
  // double voltage = readVoltage(VOLTAGE_PIN);
  // Serial.println(voltage);

  read_voltage();

  
  display.drawString(0, 1, "ABC defg");

  // display.setFont(u8g2_font_px437wyse700b_2x2_r);
  display.drawString(0, 2, "ABC defg");

  delay(2000);
}