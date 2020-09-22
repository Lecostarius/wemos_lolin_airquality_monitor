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

// for the PMS7003:
#define RX_PIN 25                                         // Rx pin which the PMS7003 Tx pin is attached to
#define TX_PIN 26                                         // Tx pin which the PMS7003 Rx pin is attached to
#define BAUDRATE 9600                                     // Device to PMS7003 Serial baudrate (should not be changed)
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
HardwareSerial mySerial(1);
SoftWire swi;

int pm1=0, pm25=0, pm10=0;
int CO2, TEMP, HYG;


void setup()
{
  Serial.begin(115200);
  Serial.println("SCD30 Example 1");
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);                      // 10, 16, 24 exist
  display.drawString(10,10,"Lecostarius");
  display.display();

  
  swi.begin(13,15);
  
  airSensor.setDebug(scd_debug);

  //This will cause readings to occur every two seconds
  if (! airSensor.begin(swi))  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
    while (1);
  }

  // display device info
  DeviceInfo();
  mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);
}

void loop() {
  char puf[128];
  uint8_t rxbuffer[34];
  int i;
  
  if (readPMS7003(rxbuffer) == 32) {
    if (rxbuffer[0] == 66 && rxbuffer[1] == 77) {
      pm1 = rxbuffer[10] * 256 + rxbuffer[11];
      pm25 = rxbuffer[12] * 256 + rxbuffer[13];
      pm10 = rxbuffer[14] * 256 + rxbuffer[15];
      //Serial.print(pm1);
      //Serial.print(",");
      //Serial.print(pm25);
      //Serial.print(",");
      //Serial.print(pm10);
      //Serial.println();
    }
  }

  if (airSensor.dataAvailable())  {
    CO2 = airSensor.getCO2();
    TEMP = airSensor.getTemperature();
    HYG = airSensor.getHumidity();
    
    Serial.print("co2(ppm):");
    Serial.print(CO2);
    sprintf(puf,"%d / %d",CO2, pm10);
    display.clear();
    display.setFont(ArialMT_Plain_24);
    display.drawString(1,0,puf);
    display.setFont(ArialMT_Plain_10);
    display.drawString(1,24,"ppm CO2 /    PM10");
    
    Serial.print(" temp(C):");
    Serial.print(TEMP, 1);
    Serial.print(" PM1/2.5/10:");
    Serial.print(pm1); Serial.print("/");Serial.print(pm25);Serial.print("/"); Serial.print(pm10);
    sprintf(puf,"PM1/2.5/10:%d/%d/%d", pm1, pm25,pm10);
    
    display.drawString(1,40,puf);
    //display.setFont(ArialMT_Plain_16);

    
    Serial.print(" humidity(%):");
    Serial.print(HYG, 1);
    sprintf(puf,"%d %%relF, %d C", HYG, TEMP);
    display.drawString(1,50,puf);
    display.display();
    
    Serial.println();

    
  } else {
    // Serial.println("No data");
  }


  delay(250);
}

int readPMS7003(uint8_t *p) {
  int i=0;
  uint8_t byteRead;
  while (mySerial.available() > 0) {
    byteRead = mySerial.read();
    if (i < 33) { p[i] = byteRead; }
    i=i+1;
  }
  return(i);
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
