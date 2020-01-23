#include <Arduino.h>

#define HAS_LORA 1         // comment out if device shall not send data via LoRa
#define CFG_sx1276_radio 1 // HPD13A LoRa SoC

#define HAS_DISPLAY U8X8_SSD1306_128X64_NONAME_HW_I2C
#define HAS_LED NOT_A_PIN // on-board LED is wired to SCL (used by display) therefore totally useless

// disable brownout detection (needed on TTGOv2 for battery powered operation)
#define DISABLE_BROWNOUT 1 // comment out if you want to keep brownout feature

// Pins for I2C interface of OLED Display
#define MY_OLED_SDA (21)
#define MY_OLED_SCL (22)
#define MY_OLED_RST U8X8_PIN_NONE

// Pins for LORA chip SPI interface come from board file, we need some
// additional definitions for LMIC
// #define LORA_RST  LMIC_UNUSED_PIN
#define LORA_IO1 (33)
#define LORA_IO2 LMIC_UNUSED_PIN

const byte VOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 12;    // 10 - 12 bits


uint16_t read_voltage()
{
  uint32_t adc_reading = analogRead(VOLTAGE_PIN);;
  Serial.print(F("ADC="));
  Serial.println(adc_reading);
  // Convert ADC reading to voltage in deciVolt, 1024/2048/4096 not hardcoded but calculated depending on the set ADC_BITS

  double voltage = adc_reading * 1100 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096)
  
  // Serial.print(F(", Voltage="));
  // Serial.println(voltage);
  
  return voltage;
}


double readVoltage(byte pin);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("ADC Measurements on the ESP32"));

  analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_0db);  // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
  pinMode(VOLTAGE_PIN, INPUT);
}

void loop()
{
  // double voltage = readVoltage(VOLTAGE_PIN);  
  double voltage = read_voltage();    
  Serial.println(voltage);
  delay(2000);
}