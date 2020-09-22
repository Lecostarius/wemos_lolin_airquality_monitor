/*
  Reading CO2, humidity and temperature from the SCD30
  By: Nathan Seidle
  SparkFun Electronics
  Date: May 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14751

  This example prints the current CO2 level, relative humidity, and temperature in C.

  Hardware Connections:
  If needed, attach a Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the device into an available Qwiic port
  Open the serial monitor at 115200 baud to see the output

  **********************************************************************
    Versioning:
  **********************************************************************
  august 2018 / paulvha:
    Support ESP8266-Thing
    include option to debug driver

  January 2019 / paulvha
    Added SoftWire support for ESP32
  **********************************************************************
    pin layout SCD30
  VDD       1 Supply Voltage ( !!! ON THE CORNER OF THE BOARD !!)
  GND       2 Ground
  TX/SCL    3 Transmission line Modbus / Serial clock I2C
  RX/SDA    4 Receive line Modbus / Serial data I2C
  RDY       5 Data ready. High when data is ready for read-out  (*1)
  PWM       6 PWM output of CO2 concentration measurement  (*1)
              (May2020 : supported  BUT not implemented)
  SEL       7 Interface select pin. Pull to VDD for selecting Modbus,
              leave floating or connect to GND for selecting I2C. (*1)

  Note *1 : none of these lines are connected or used.

  CONNECT TO ARDUINO

  SCD30 :
    VCC to 3V3 or 5V
    GND to GND
    SCL to SCL ( Arduino UNO A4)
    SDA to SDA ( Arduino UNO A5)

  CONNECT TO ESP8266-THING

  Make sure to cut the link and have a jumper on the DTR/reset.
  Include the jumper for programming, remove before starting serial monitor

  SCD30    ESP8266
    GND --- GND
    VCC --- 3V3
    SCL --- SCL
    SDA --- SDA

  Given that SCD30 is using clock stretching the driver has been modified to deal with that.

  CONNECT TO ESP32-THING

  SCD30    ESP32
    GND --- GND
    VCC --- 3V3
    SCL --- 22
    SDA --- 21

  Given that SCD30 is using clock stretching SoftWire is selected by the driver to deal with that.
  Make sure to press the GPIO0 button for connect /upload
*/

//////////////////////////////////////////////////////////////////////////
// set SCD30 driver debug level (only NEEDED case of errors)            //
// Requires serial monitor (remove DTR-jumper before starting monitor)  //
// 0 : no messages                                                      //
// 1 : request sending and receiving                                    //
// 2 : request sending and receiving + show protocol errors             //
//////////////////////////////////////////////////////////////////////////
#define scd_debug 0

//////////////////////////////////////////////////////////////////////////
//                SELECT THE WIRE INTERFACE                             //
//////////////////////////////////////////////////////////////////////////
//#define SCD30WIRE Wire

// for the MHZ19:
#define RX_PIN 13                                          // Rx pin which the MHZ19 Tx pin is attached to
#define TX_PIN 15                                          // Tx pin which the MHZ19 Rx pin is attached to
#define BAUDRATE 9600                                      // Device to MH-Z19 Serial baudrate (should not be changed)
#define NO_GLOBAL_SOFTWIRE

// for the SCD30:
#define SDA_PIN 2
#define SCL_PIN 14

#include <Arduino.h>
//#include "MHZ19.h"
//#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or Arduino package without SoftwareSerial support


#include "paulvha_SCD30.h"
#include "SSD1306.h"

SCD30 airSensor;
SSD1306 display(0x3c, 5, 4);
//MHZ19 myMHZ19;                                             // Constructor for library
//HardwareSerial mySerial(1);
SoftWire swi;

void setup()
{
  Serial.begin(115200);
  Serial.println("SCD30 Example 1");
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);                      // 8, 16, 24 exist
  display.drawString(10,10,"Lecostarius");
  display.display();

  //SCD30WIRE.begin(); // default 21, 22
  //twi.begin(2, 14);
  //SoftWire.begin(2, 14); // sda, scl 
  swi.begin(13,15);
  
  airSensor.setDebug(scd_debug);

  //This will cause readings to occur every two seconds
  if (! airSensor.begin(swi))
  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while (1);
  }

  // display device info
  DeviceInfo();
  
}

void loop() {
  char puf[22];

  if (airSensor.dataAvailable())
  {
    int CO2, TEMP, HYG;
    CO2 = airSensor.getCO2();
    Serial.print("co2(ppm):");
    Serial.print(CO2);
    sprintf(puf,"CO2: %d ppm",CO2);
    display.clear();
    display.drawString(1,0,puf);
    
    TEMP = airSensor.getTemperature();
    Serial.print(" temp(C):");
    Serial.print(TEMP, 1);
    sprintf(puf,"T  : %d C", TEMP);
    display.drawString(1,18,puf);

    HYG = airSensor.getHumidity();
    Serial.print(" humidity(%):");
    Serial.print(HYG, 1);
    sprintf(puf,"HYG: %d %%rel", HYG);
    display.drawString(1,36,puf);
    display.display();
    
    Serial.println();
    
  }
  else
    Serial.println("No data");

  delay(2000);
}

void DeviceInfo()
{
  uint8_t val[2];
  char buf[(SCD30_SERIAL_NUM_WORDS * 2) + 1];

  // Read SCD30 serial number as printed on the device
  // buffer MUST be at least 33 digits (32 serial + 0x0)
  if (airSensor.getSerialNumber(buf))
  {
    Serial.print(F("SCD30 serial number : "));
    Serial.println(buf);
  }

  // read Firmware level
  if ( airSensor.getFirmwareLevel(val) ) {
    Serial.print("SCD30 Firmware level: Major: ");
    Serial.print(val[0]);

    Serial.print("\t, Minor: ");
    Serial.println(val[1]);
  }
  else {
    Serial.println("Could not obtain firmware level");
  }
}
