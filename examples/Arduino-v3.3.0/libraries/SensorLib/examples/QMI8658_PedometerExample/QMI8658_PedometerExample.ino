/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      QMI8658_PedometerExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-26
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SensorQMI8658.hpp"
#ifdef ARDUINO_T_BEAM_S3_SUPREME
#include <XPowersAXP2101.tpp>   //PMU Library https://github.com/lewisxhe/XPowersLib.git
#endif


// #define USE_I2C              //Using the I2C interface

#ifdef USE_I2C
#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#else   /* SPI interface */

#ifndef SPI_MOSI
#define SPI_MOSI   (35)
#endif

#ifndef SPI_SCK
#define SPI_SCK    (36)
#endif

#ifndef SPI_MISO
#define SPI_MISO   (37)
#endif

#endif  /* USE_I2C*/

#ifndef IMU_CS
#define IMU_CS      34      // IMU CS PIN
#endif

#ifndef IMU_IRQ
#define IMU_IRQ     33      // IMU INT PIN
#endif

#ifndef OLED_SDA
#define OLED_SDA    22      // Display Wire SDA Pin
#endif

#ifndef OLED_SCL
#define OLED_SCL    21      // Display Wire SCL Pin
#endif

SensorQMI8658 qmi;

bool interruptFlag = false;


void beginPower()
{
    // T_BEAM_S3_SUPREME The PMU voltage needs to be turned on to use the sensor
#if defined(ARDUINO_T_BEAM_S3_SUPREME)
    XPowersAXP2101 power;
    power.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41);
    power.disableALDO1();
    power.disableALDO2();
    delay(250);
    power.setALDO1Voltage(3300); power.enableALDO1();
    power.setALDO2Voltage(3300); power.enableALDO2();
#endif
}

void setFlag(void)
{
    interruptFlag = true;
}


void pedometerEvent()
{
    uint32_t val = qmi.getPedometerCounter();
    Serial.print("Detected Pedometer event : ");
    Serial.println(val);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    beginPower();

    bool ret = false;
#ifdef USE_I2C
    ret = qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL);
#else
#if defined(SPI_MOSI) && defined(SPI_SCK) && defined(SPI_MISO)
    ret = qmi.begin(SPI, IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK);
#else
    ret = qmi.begin(SPI, IMU_CS);
#endif
#endif

    if (!ret) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);

    // Equipped with acceleration sensor, 2G, ORR62.5HZ
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_2G, SensorQMI8658::ACC_ODR_62_5Hz);

    // Enable the accelerometer
    qmi.enableAccelerometer();

    //* Indicates the count of sample batch/window for calculation
    uint16_t ped_sample_cnt = 50; //50 samples
    //* Indicates the threshold of the valid peak-to-peak detection
    uint16_t ped_fix_peak2peak = 200;   //200mg
    //* Indicates the threshold of the peak detection comparing to average
    uint16_t ped_fix_peak = 100;    //100mg
    //* Indicates the maximum duration (timeout window) for a step.
    //* Reset counting calculation if no peaks detected within this duration.
    uint16_t ped_time_up = 200; // 200 samples 4s
    //* Indicates the minimum duration for a step.
    //* The peaks detected within this duration (quiet time) is ignored.
    uint8_t ped_time_low = 20; //20 samples
    //*   Indicates the minimum continuous steps to start the valid step counting.
    //*   If the continuously detected steps is lower than this count and timeout,the steps will not be take into account;
    //*   if yes, the detected steps will all be taken into account and counting is started to count every following step before timeout.
    //*   This is useful to screen out the fake steps detected by non-step vibrations
    //*   The timeout duration is defined by ped_time_up.
    uint8_t ped_time_cnt_entry = 10; //10 steps entry count
    //*   Recommended 0
    uint8_t ped_fix_precision = 0;
    //*   The amount of steps when to update the pedometer output registers.
    uint8_t ped_sig_count = 4; //Every 4 valid steps is detected, update the registers once (added by 4).

    qmi.configPedometer(ped_sample_cnt,
                        ped_fix_peak2peak,
                        ped_fix_peak,
                        ped_time_up,
                        ped_time_low,
                        ped_time_cnt_entry,
                        ped_fix_precision,
                        ped_sig_count);

    // Enable the step counter and enable the interrupt
    if (!qmi.enablePedometer(SensorQMI8658::INTERRUPT_PIN_1)) {
        Serial.println("Enable pedometer failed!");
        while (1);
    }

    // Set the step counter callback function
    qmi.setPedometerEventCallBack(pedometerEvent);

    /*
     * When the QMI8658 is configured as Wom, the interrupt level is arbitrary,
     * not absolute high or low, and it is in the jump transition state
     */
    attachInterrupt(IMU_IRQ, setFlag, CHANGE);
}


void loop()
{
    if (interruptFlag) {
        interruptFlag = false;
        qmi.update();
    }
    delay(500);
}