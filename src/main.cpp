#include <Arduino.h>


#define HAS_LORA 1       // comment out if device shall not send data via LoRa
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
#define LORA_IO1  (33)
#define LORA_IO2  LMIC_UNUSED_PIN


const byte ADCBOARDVOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 10;            // 10 - 12 bits

uint16_t read_voltage()
{
  // multisample ADC
  const byte NO_OF_SAMPLES = 5;
  uint32_t adc_reading = 0;
  Serial.print(F("D022 ADC Measurement:"));
  analogRead(ADCBOARDVOLTAGE_PIN); // First measurement has the biggest difference on my board, this line just skips the first measurement
  for (int i = 0; i < NO_OF_SAMPLES; i++)
  {
    uint16_t thisReading = analogRead(ADCBOARDVOLTAGE_PIN);
    adc_reading += thisReading;
    Serial.print(F(" "));
    Serial.print(thisReading);
  }
  adc_reading /= NO_OF_SAMPLES;
  Serial.print(F(" Avg="));
  Serial.println(adc_reading);
  // Convert ADC reading to voltage in deciVolt, 1024/2048/4096 not hardcoded but calculated depending on the set ADC_BITS
  uint16_t voltage = adc_reading * 2200 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096)
  return voltage;
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("ADC Measurements on the ESP32"));

  analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_6db); // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
  pinMode(ADCBOARDVOLTAGE_PIN, INPUT);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.print(F("Voltage: "));
  Serial.println(read_voltage());
  delay(2000);
}