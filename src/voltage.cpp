#include <Arduino.h>

String readBatteryPercentage(byte pin)
{
    // see https://github.com/G6EJD/LiPo_Battery_Capacity_Estimator/blob/master/ReadBatteryCapacity_LIPO.ino
    uint8_t percentage = 100;
    float voltage = analogRead(pin) / 4096.0 * 7.23; // LOLIN D32 (no voltage divider need already fitted to board.or NODEMCU ESP32 with 100K+100K voltage divider
    //float voltage = analogRead(39) / 4096.0 * 7.23;    // NODEMCU ESP32 with 100K+100K voltage divider added
    //float voltage = analogRead(A0) / 4096.0 * 4.24;    // Wemos / Lolin D1 Mini 100K series resistor added
    //float voltage = analogRead(A0) / 4096.0 * 5.00;    // Ardunio UNO, no voltage divider required
    Serial.println("Voltage = " + String(voltage));
    percentage = 2808.3808 * pow(voltage, 4) - 43560.9157 * pow(voltage, 3) + 252848.5888 * pow(voltage, 2) - 650767.4615 * voltage + 626532.5703;
    if (voltage > 4.19)
        percentage = 100;
    else if (voltage <= 3.50)
        percentage = 0;
    return String(percentage) + "%";
}

/**
 * https://github.com/G6EJD/ESP32-ADC-Accuracy-Improvement-function
 */
double readVoltage(byte pin)
{
    double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
    if (reading < 1 || reading > 4095)
        return 0;
    // return -0.000000000009824 * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
    return -0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089;
} // Added an improved polynomial, use either, comment out as required

/* ADC readings v voltage
 *  y = -0.000000000009824x3 + 0.000000016557283x2 + 0.000854596860691x + 0.065440348345433
 // Polynomial curve match, based on raw data thus:
 *   464     0.5
 *  1088     1.0
 *  1707     1.5
 *  2331     2.0
 *  2951     2.5 
 *  3775     3.0
 *  
 */
