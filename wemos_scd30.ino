/*
 --------------------------------------------------------------------------------------------------------
 by Lecostarius
  Air quality monitoring with a Wemos Lolin with OLED (ESP32 WROOM w/ a 128x64 OLED driven by SSD1306)

  Three sensors are attached:
    - a SCD30 from Sensirion for CO2, Temperature, Humidity,
    - a PMS7003 particle sensor (give PM1, PM2.5, PM10 fine dust),
    - a MiCS 6814 triple chemical sensor (for NH3, CO, NO2), with breakout from Seeed
  One display is attached:
    - a 128x64 OLED with a SSD1306 driver chip

    The SCD30 and the MiCS6814 and the display are all three connected using I2C. The respective 
    adresses are 0x04 for the MiCS6814 Seeed breakout, 0x61 for the SCD30, and 0x3c for the display.
    However, the requirements of the three I2C bus slaves are very different. 
    The display is hard-connected to GPIO 4 (SCL) and GPIO 5 (SDA). It needs to run at the highest
     possible speed.
    The SCD30, on the contrary, has an absolute maximum I2C speed of 100 kHz and uses clock stretching
    a lot. According to the datasheet, it can delay the transfer by up to 150 ms which amounts to 7 Hz.
    This happens rarely, but the fast display I2C speed and the extreme bus usage of the SCD30 do not
    play well together. So I decided to put the SCD30 on another I2C bus (which is software emulated,
    because the ESP32 does not support clock stretching of 150 ms...)

    The MiCS6814 Seeed breakout board has no documentation about its I2C limitations. Likely it does
    not support the very fast I2C that the display runs, so it can be put on the same hardware pins as the
    SCD30. The problem with it is that the library provided by Seeed uses the "Wire" object throughout to 
    communicate with the uC they put on their breakout board, and the "Wire" object is already used by
    the SSD1306 display driver software. I still need to find a solution for that, other than rewriting 
    the entire Seeed library "MutichannelGasSensor.cpp|h"
    
    The SCD30 board has pull-up resistors (pretty high ones, but the speed is low, so it is OK)
    so no external components are required for neither the SCD30 nor the MiCS6814 which is on the same
    two pins. 

    The PMS7003 uses a serial connection and regularly broadcasts its data over serial.
    We use the hardware serial #1 of the ESP32 for that.

    The code for the SCD30 is the code from paulvha. Slight modifications were required to allow
    running the I2C not from the standard pins.
    The SoftWire library is also taken from paulvha; it has a long version history dating back to 
    an original Arduino version of 2006.
    The code for the MiCS 6814 is taken almost literally from Seeed (MutichannelGasSensor). Since this
    code includes <Wire.h> but we can not do it (we need to use SoftWire) the Seeed code had to be
    changed by commenting out the #include <Wire.h>. This is the only required change. The Wire object
    is instead instantiated by an include in paulvha_SCD30.h.

    The code for the PMS7003 is trivial and written by myself.

    The code for the SSD1306 is the library written by Fabrice Weinberg and ThingPulse available via the
    Arduino library facility: "ESP8266 and ESP32 OLED driver for SSD1306 displays"
    
    All copyright texts are left in the respective files, including this one (follows)
    Any code added by me to this project is freeware. Use for whatever purpose.
  ------------------------------------------------------------------------------------------------------


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

// for the SCD30 and MiCS6814:

#define SDA_PIN 2
#define SCL_PIN 14

#include <Arduino.h>
//#include <SoftwareSerial.h>                                // Remove if using HardwareSerial or Arduino package without SoftwareSerial support

#define NO_GLOBAL_SOFTWIRE
#include "paulvha_SCD30.h"  // this includes SoftWire.h but with NO_GLOBAL_SOFTWIRE will not create a "Wire" object
#include "SSD1306.h"        // this might include Wire.h

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
