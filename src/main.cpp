#include <Arduino.h>
#include <CayenneLPP.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include "EEPROM.h"

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
* https://github.com/LilyGO/TTGO-LORA32
* https://github.com/bertrik/LoraWanPmSensor/blob/master/Esp32PmSensor/Esp32PmSensor.ino
*/

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = {0x1F, 0x9C, 0x02, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = {0xC5, 0xE7, 0x9F, 0x1B, 0x29, 0xBB, 0x13, 0x00};
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = {0x1D, 0x48, 0xB6, 0xFA, 0xBE, 0x11, 0xBC, 0xC1, 0x64, 0x02, 0x8B, 0x0F, 0x0E, 0x9F, 0x49, 0x90};
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

typedef struct
{
  u4_t netid = 0;
  devaddr_t devaddr = 0;
  u1_t nwkKey[16];
  u1_t artKey[16];
  uint32_t magic;
} otaa_data_t;
#define OTAA_MAGIC 0xCAFEBABE

#define CFG_sx1276_radio 1 // HPD13A LoRa SoC
const unsigned TX_INTERVAL = 20;

// #define HAS_LED NOT_A_PIN // on-board LED is wired to SCL (used by display) therefore totally useless

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

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    //.rst = LMIC_UNUSED_PIN,
    .dio = {/*dio0*/ 26, /*dio1*/ 33, /*dio2*/ 32}};

static osjob_t sendjob;
CayenneLPP lpp(51);
char buff[20];

void readSensorValues();
void readDownloadData();
void do_send(osjob_t *j);
void loop();
void setupSensor();

const byte VOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 12;    // 10 - 12 bits

float voltage = 0;
float height = 0;
uint16_t adc_reading = 0;
static otaa_data_t otaa_data;

float calcHeigth()
{
  float height = 48.7 * voltage - 17.3;
  return -height;
}

void printValues()
{
  char buff[16];
  u8x8.inverse();
  snprintf(buff, sizeof(buff), "U:%.2fV", voltage);
  u8x8.draw2x2String(0, 0, buff);

  u8x8.noInverse();
  // u8x8.setCursor(0, 3);
  // u8x8.printf("ADC:%-4d", adc_reading);

  snprintf(buff, sizeof(buff), "ADC:%-4d", adc_reading);
  u8x8.draw2x2String(0, 3, buff);

  snprintf(buff, sizeof(buff), "%.1fcm", height);
  u8x8.draw2x2String(0, 5, buff);

  Serial.printf("U=%.2fV\t\tADC=%-4d\t\tHeight=:%.1fcm\n", voltage, adc_reading, height);
}

void readSensorValues()
{
  adc_reading = analogRead(VOLTAGE_PIN);
  voltage = adc_reading * 2.20 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096)
  height = calcHeigth();

  printValues();

  lpp.reset();
  lpp.addAnalogInput(1, voltage);
  lpp.addTemperature(2, height);
}

static void print_keys(void)
{
  Serial.print("netid: ");
  Serial.println(otaa_data.netid, DEC);
  Serial.print("devaddr: ");
  Serial.println(otaa_data.devaddr, HEX);
  Serial.print("artKey: ");
  for (int i = 0; i < sizeof(otaa_data.artKey); ++i)
  {
    Serial.printf("%02X", otaa_data.artKey[i]);
  }
  Serial.println("");
  Serial.print("nwkKey: ");
  for (int i = 0; i < sizeof(otaa_data.nwkKey); ++i)
  {
    Serial.printf("%02X", otaa_data.nwkKey[i]);
  }
  Serial.println("");
}

void storeKeysInEeprom()
{
  LMIC_getSessionKeys(&otaa_data.netid, &otaa_data.devaddr, otaa_data.nwkKey,
                      otaa_data.artKey);
  otaa_data.magic = OTAA_MAGIC;
  EEPROM.put(0, otaa_data);
  EEPROM.commit();
}

void onEvent(ev_t ev)
{
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev)
  {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));

    storeKeysInEeprom();
    print_keys();

    // Disable link check validation (automatically enabled
    // during join, but because slow data rates change max TX
    // size, we don't use it in this example.
    LMIC_setLinkCheckMode(0);
    Serial.println(F("Join ok"));
    break;
  /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen)
    {
      Serial.print(F("Received "));
      Serial.print(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));

      readDownloadData();
    }

    // Schedule next transmission
    // os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);

    esp_err_t err;
    err = esp_sleep_enable_timer_wakeup(2 * 60 * 1000000LL);
    Serial.print(F("err="));
    Serial.println(err);
    Serial.println(F("Will sleep now ..."));
    esp_deep_sleep_start();

    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
  case EV_TXSTART:
    Serial.println(F("EV_TXSTART"));
    break;
  default:
    Serial.print(F("Unknown event: "));
    Serial.println((unsigned)ev);
    break;
  }
}

void do_send(osjob_t *j)
{
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("OP_TXRXPEND, not sending"));
  }
  else
  {
    readSensorValues();
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 1);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void loop()
{
  os_runloop_once();
}

void readDownloadData()
{
  snprintf(buff, sizeof(buff), "RSSI = %d dB", LMIC.rssi);
  Serial.println(buff);
  u8x8.drawString(0, 3, buff);

  snprintf(buff, sizeof(buff), "SNR = %.1d dB", LMIC.snr);
  Serial.println(buff);
  u8x8.drawString(0, 4, buff);

  snprintf(buff, sizeof(buff), "RX = %d bytes", LMIC.dataLen);
  Serial.println(buff);
  u8x8.drawString(0, 5, buff);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ADC Measurements on the ESP32");

  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clear();
  u8x8.draw2x2String(0, 0, "Water");
  u8x8.draw2x2String(0, 3, "Level");
  u8x8.draw2x2String(0, 6, "Meter");
  delay(1000);
  u8x8.clear();

  analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_6db);  // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
  pinMode(VOLTAGE_PIN, INPUT);

  // LMIC init
  os_init();
  LMIC_reset();
  EEPROM.begin(512);
  EEPROM.get(0, otaa_data);
  // if (otaa_data.magic == OTAA_MAGIC)
  // {
  //   LMIC_setSession(otaa_data.netid, otaa_data.devaddr, otaa_data.nwkKey, otaa_data.artKey);
  //   Serial.println(F("Keys loaded from EEPROM:"));
  //   print_keys();
  // }
  // else
  // {
    LMIC_startJoining();
  // }

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}