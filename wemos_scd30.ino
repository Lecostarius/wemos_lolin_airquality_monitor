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
    the SSD1306 display driver software. Therefore, the Seeed library (MutichannelGasSensor (sic)) has been
    modified such that it does not use hardcoded the Wire object, but rather uses TWOWIRE which is a preprocessor
    macro and could be set to "Wire" - then, the library behaves as before - but also to the name of a
    SoftWire object (in my case, "swi"). I also removed the Wire.begin() call from the library - this should
    be done by the main program anyway, since the I2C could have multiple devices attached (like in my case).
    
    The SCD30 board has pull-up resistors (pretty high ones, but the speed is low, so it is OK)
    so no external components are required for neither the SCD30 nor the MiCS6814 which is on the same
    two pins. 

    The PMS7003 uses a serial connection and regularly broadcasts its data over serial.
    We use the hardware serial #1 of the ESP32 for that.

    The code for the SCD30 is the code from paulvha. Slight modifications were required to allow
    running the I2C not from the standard pins. Also, the begin() method had to be changed to allow the
    use of SoftWire together with a parallel use of Wire.h (mandated by the SSD1306).
    
    The SoftWire library is also taken from paulvha; it has a long version history dating back to 
    an original Arduino version of 2006.
    The code for the MiCS 6814 is taken almost literally from Seeed (MutichannelGasSensor). The required
    changes are mentioned in the comment above. The original name "MutichannelGasSensor" has been changed
    to "Mics6814".

    The code for the PMS7003 is trivial and written by myself.

    The code for the SSD1306 is the library written by Fabrice Weinberg and ThingPulse available via the
    Arduino library facility: "ESP8266 and ESP32 OLED driver for SSD1306 displays"
    
    All copyright texts are left in the respective files, including this one (follows)
    Any code added by me to this project is freeware. Feel free to use for whatever purpose.

    Connecting the Hardware

    As for power, the PMS7003 runs on 5 V power (the fan needs 5V), the SCD30 and the Mics6814 breakout both
    run either on 3.3V or 5 V. The Mics6814 draws a maximum of 150mW of power even when heating, which is less
    than 50 mA, so I think it can safely be connected to the 3.3V power pin of the Wemos Lolin board.
    The entire board - with sensors attached - seems to consume between 120mA and 180mA of current at 5V USB
    supply. For a 10000mAh power bank that translates into roughly 2 days of continuous operation.
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
// print sensor results and diagnostics also over Serial?
#define PRINT_VIA_SERIAL 1

// for the PMS7003:
#define RX_PIN 25                                         // Rx pin which the PMS7003 Tx pin is attached to
#define TX_PIN 0                                          // not used by us, we only receive
#define BAUDRATE 9600                                     // Device to PMS7003 Serial baudrate (should not be changed)

// for the SCD30 and MiCS6814:

#define SDA_PIN 13
#define SCL_PIN 15

// for the Buzzer:
#define BUZZERPIN 16

// for switching on/off the PMS7003 (to increase its lifetime):
#define PMS7003PIN 2

// calibration input pin: if this is pulled low, calibrate the MiCS6814:
#define CALIBRATIONPIN 26 

// DECAY is between 0 and 0.999, and is the forgetting factor for the CO concentration
// integrator. The higher, the longer the integration constant. 
#define DECAY 0.99
#define WARNLEVEL1 100 // ppm CO when we warn immediately
#define WARNLEVEL2 7   // ppm CO if active over a long time

// if you #define it, it will print info via Serial
#undef PRINT_VIA_SERIAL 

// ************************************
// #include section
// ************************************
#include <Arduino.h>
#define NO_GLOBAL_SOFTWIRE
#include "paulvha_SCD30.h"  // this includes SoftWire.h but with NO_GLOBAL_SOFTWIRE will not create a "Wire" object
#include "SSD1306.h"        // this will include Wire.h, and create a Wire object which is used to communicate with the display.
#include "Mics6814.h"
#include <WiFi.h>
#include <WebServer.h>

// ************************************
// Global variables
// ************************************
int pm1=0, pm25=0, pm10=0;
int CO2, TEMP, HYG;
float concentrationNH3, concentrationCO, concentrationNO2, concentrationAlc;
int displayType=0, displayCtr=0;
int cycle = 0; // cyclic counter controlling on/off of the dust sensor, averaging of values etc
uint32_t gcycle = 0; // global cycle counter, counts time since start
float co_integral = 0.0; // integrated CO concentration over time
char puf[128];
uint8_t rxbuffer[34];
// webserver stuff:
// SSID & Password
const char* ssid = "ESP32";  // Enter your SSID here
const char* password = "123456789";  //Enter your Password here
// IP Address details
IPAddress local_ip(192, 168, 3, 1);
IPAddress gateway(192, 168, 3, 1);
IPAddress subnet(255, 255, 255, 0);

// **************************************
// Create global objects
// **************************************
HardwareSerial mySerial(1);
SoftWire swi;
SCD30 airSensor;              // the constructor does not yet talk to the device. This happens during begin().
SSD1306 display(0x3c, 5, 4);  // on the Wemos Lolin board, the OLED is connected via I2C using pins 4 (SCL) and 5 (SDA).
Mics6814 mics;                // the library knows the I2C adress 0x04 of the Seeed breakout board. This constructor does not yet
                              // talk with the device. The device is adressed using the begin() method.
WebServer server(80);  // Object of WebServer(HTTP port, 80 is default)

void setup() {
  // Serial is used for communication with the host PC over USB, mostly for diagnosis and debugging
  // this is sent to nowhere once the device functions autonomously
  Serial.begin(115200);
  Serial.println("Air Quality Sensor v1.0");

  pinMode(CALIBRATIONPIN, INPUT_PULLUP);
  pinMode(BUZZERPIN, OUTPUT);
  pinMode(PMS7003PIN, OUTPUT);
  digitalWrite(BUZZERPIN, LOW);
  digitalWrite(PMS7003PIN, HIGH);
  
  // set up sequence of the display. Uses I2C connection over pins 4,5 and the Wire object.
  display.init();
  display.clear();
  display.setFont(ArialMT_Plain_16);                      // 10, 16, 24 exist
  display.drawString(10,10,"(c) Lecostarius");
  display.setFont(ArialMT_Plain_10); 
  display.drawString(0,30,"WiFi: 'ESP32'");
  display.drawString(0,40,"IP  : 192.168.3.1");
  display.display();

  // set up the second I2C interface, using pins SDA_PIN, SCL_PIN and a bit-banging library
  swi.begin(SDA_PIN,SCL_PIN);         

  // *** set up sequence of the SCD30 CO2 sensor ***
  airSensor.setDebug(scd_debug);
  if (! airSensor.begin(swi))  {
    Serial.println(F("The SCD30 did not respond. Please check wiring."));
  }
  // query SCD30 for version and firmware and print results using Serial:
  DeviceInfo();
  
  
  // *** set up sequence of the Mics6814 multichannel gas sensor ***
  mics.begin(0x04);   // the usage of the "swi" object for I2C comm is hard-wired into the library
  Serial.print("mics6814 version: ");
  Serial.println(mics.getVersion());
  mics.powerOn(); 
  Serial.println("mics6814 power on");
  mics.ledOn();
  

  // *** set up sequence for the fine dust sensor PMS7003 ***
  // not really doing anything - this device just sends data over serial, so lets start the serial
  // port that receives the data
  mySerial.begin(BAUDRATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // *** set up sequence for the WiFi ***
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  server.on("/", handle_root);
  server.begin();
 #ifdef PRINT_VIA_SERIAL
  Serial.print("HTTP server started. IP is ");
  Serial.println(local_ip);
  Serial.print("Password: ");
  Serial.println(password);
 #endif
  delay(100);
  
}

void loop() {
  int i;
  

  cycle  = cycle + 1; if (cycle > 100) cycle = 0;
  gcycle = gcycle + 1;
  
  // switch on dust sensor, or switch it off:
  if (cycle < 50) { 
    digitalWrite(PMS7003PIN, HIGH);
  } else {
    digitalWrite(PMS7003PIN, LOW);
  }
  
  // *** PMS7003 fine dust sensor ***
  if (readPMS7003(rxbuffer) == 32) {
    if (rxbuffer[0] == 66 && rxbuffer[1] == 77) {
      // 0x6677 is a magic string the PMS7003 sends. We use it to verify that this is indeed data
      //  from the sensor and that it is valid.
      if (cycle > 25) {
        pm1 = rxbuffer[10] * 256 + rxbuffer[11];
        pm25 = rxbuffer[12] * 256 + rxbuffer[13];
        pm10 = rxbuffer[14] * 256 + rxbuffer[15];
      }
    }
  }

  // *** SCD30 CO2 sensor ***
  if (airSensor.dataAvailable())  {
    CO2 = airSensor.getCO2();
    TEMP = airSensor.getTemperature();
    HYG = airSensor.getHumidity();
    //displayCO2(CO2, TEMP, HYG, pm1, pm25, pm10);
  } 

  // *** MiCS6814 chemical sensor ***
  concentrationNH3 = mics.measure_NH3();
  concentrationCO  = mics.measure_CO();
  concentrationNO2 = mics.measure_NO2();
  concentrationAlc = mics.measure_C2H5OH();
  
 #ifdef PRINT_VIA_SERIAL
  Serial.print("The concentration of NH3 is [ppm]"); Serial.println(concentrationNH3);
  Serial.print("The concentration of NO2 is [ppm]"); Serial.println(concentrationNO2);
  Serial.print("The concentration of CO is [ppm]"); Serial.println(concentrationCO);
  Serial.print("The concentration of Alcohol is [ppm]"); Serial.println(concentrationAlc);
 #endif
  
  // in the first minute, do not look at CO concentration, sensor is still heating
  // should be 10 minutes, really
  if (gcycle > 120) {
    co_integral += concentrationCO;
    co_integral = co_integral * DECAY;
#ifdef PRINT_VIA_SERIAL
    Serial.print("co_integral, modified integral, conc: "); Serial.print(co_integral); Serial.print(", "); Serial.print(co_integral*(1-DECAY)); Serial.print(", "); 
    Serial.println(concentrationCO);
#endif  
    if (concentrationCO > WARNLEVEL1 || co_integral*(1-DECAY) > WARNLEVEL2) {
      co_integral = co_integral * 0.95; // decay a bit - we issued a warning. The constant is somewhat arbitrary here.
      digitalWrite(BUZZERPIN, HIGH);
      delay(500); // beep a little
      digitalWrite(BUZZERPIN, LOW);
    }
  }
  // done reading, display results
  displayCtr++;
  if (displayCtr > 5) {
    displayCtr = 0;
    if (displayType == 0) {
      displayType = 1;
      displayCO2(CO2, TEMP, HYG, pm1, pm25, pm10);
    } else {
      displayType = 0;
      displayChem(concentrationNH3, concentrationCO, concentrationAlc, concentrationNO2);
    }
  }

  // check whether calibration of MiCS6814 is requested (by pulling the CALIBRATIONPIN to LOW
  if (digitalRead(CALIBRATIONPIN) == LOW) {
    displayCalibMessage1();
    delay(300); // Taste entprellen
    while (digitalRead(CALIBRATIONPIN) == LOW) {
      delay(100); // wait for pin release, forever
    }
    displayCalibMessage2();
    mics.doCalibrate();
    delay(8000); // wait 8 seconds
    digitalWrite(BUZZERPIN,HIGH); delay(200); digitalWrite(BUZZERPIN,LOW);
    delay(200);
    digitalWrite(BUZZERPIN,HIGH); delay(200); digitalWrite(BUZZERPIN,LOW);
    delay(200);
    digitalWrite(BUZZERPIN,HIGH); delay(200); digitalWrite(BUZZERPIN,LOW);
  } 

  
  if (displayCtr == 4) {
    //digitalWrite(BUZZERPIN, HIGH);
  } else {
    //digitalWrite(BUZZERPIN, LOW);
  }

  // Webserver
  server.handleClient();

  
  delay(500); // all our sensors are slow. No point in reading them too often.
  /* 
   *  The PMS7003 sends its data in automatic mode with a variable speed. According to the datasheet,
   *  it will send data more frequently if the measured fine dust concentration changes quickly. The 
   *  normal rate is once every 2.3 seconds if there is little change, but if a significant change is 
   *  observed, it will switch to "fast mode" where it will send new data after an interval of 200..800 ms,
   *  depending on the rate of change. According the the article "Evaluation of Low-Cost Sensors for Ambient PM2.5 Monitoring"
   *  by Marek Badura et al, there are spurious spikes of PM2.5 concentration in this type of sensors, which
   *  should be smoothened by averaging over periods of 15 seconds. Therefore, I decided to wait longer than the
   *  minimum update time of the sensor (and therefore to skip some results), since it is likely that this results
   *  are not too accurate anyway and there is probably no point in following the quick changes. This does not improve
   *  accuracy in any way however; I should really average results over a 15 second window.
   *  
   */
}


// HTML & CSS contents which display on web server
String HTML = "<!DOCTYPE html>\
<html>\
<body>\
<h1>My First Web Server with ESP32 - AP Mode &#128522;</h1>\
</body>\
</html>";

// Handle root url (/)
void handle_root() {
  String content = "<!DOCTYPE html>\
<html>\
<head><meta http-equiv=\"refresh\" content=\"2\" ></head>\
<body>\
<h1>Luftqualit&auml;tsmonitor</h1>\
<br>\
<h2>CO2: ";
  content+=CO2;
  content += "</h2><br><h2>PM10 : ";
  content += pm10;
  content += "</h2><br><h2>PM2.5: ";
  content += pm25;
  content += "</h2><br><h2>PM1  : ";
  content += pm1;
  content += "</h2><br>";
  if (concentrationCO > 5) {
    content += "<h2><i>CO   : ";
    content += concentrationCO;
    content += "</i>";
  } else {
    content += "<h2>CO   : ";
    content += concentrationCO;
  }
  
  content += "</h2><br><h2>NO2  : ";
  content += concentrationNO2;
  content += "</body></html>";
  server.send(200, "text/html", content);
} 


void displayCalibMessage1() {
#ifdef PRINT_VIA_SERIAL
    Serial.print("??? CALIBRATION ???"); Serial.println();
    Serial.print("If you want calibration, release CALIBRATIONPIN now, else switch off"); Serial.println();
#endif
    display.clear();
    display.setFont(ArialMT_Plain_16);
    sprintf(puf,"CALIBRATION ??"); 
    display.drawString(0,0,puf); 
    display.setFont(ArialMT_Plain_10);
    sprintf(puf,"Calibration: release pin");
    display.drawString(0,40,puf);
    sprintf(puf,"Otherwise switch off");
    display.drawString(0,52,puf);
    display.display();
}

void displayCalibMessage2() {
#ifdef PRINT_VIA_SERIAL
    Serial.print("*** CALIBRATING ***"); Serial.println();
    Serial.print("Please wait"); Serial.println();
#endif
    display.clear();
    display.setFont(ArialMT_Plain_16);
    sprintf(puf,"** CALIBRATING **"); 
    display.drawString(0,0,puf); 
    display.setFont(ArialMT_Plain_10);
    sprintf(puf,"Calibration ongoing, wait 15s");
    display.drawString(0,40,puf);
    display.display();
}


void displayCO2(int CO2, int TEMP, int HYG, int pm1, int pm25, int pm10) {
#ifdef PRINT_VIA_SERIAL
    Serial.print("co2(ppm):");
    Serial.print(CO2);
    Serial.print(" temp(C):");
    Serial.print(TEMP, 1);
    Serial.print(" PM1/2.5/10:");
    Serial.print(pm1); Serial.print("/");Serial.print(pm25);Serial.print("/"); Serial.print(pm10);
    Serial.print(" humidity(%):");
    Serial.print(HYG, 1);
    Serial.println();
#endif
   
    display.clear();
    display.setFont(ArialMT_Plain_24);
    sprintf(puf,"%d",CO2); 
    display.drawString(0,0,puf); 
    display.setFont(ArialMT_Plain_10); 
    display.drawString(70,7,"ppm CO");
    int x = display.getStringWidth("ppm CO");
    display.drawString(70+x,12,"2");

    display.setFont(ArialMT_Plain_24);
    sprintf(puf,"%d",pm10);
    display.drawString(0,24,puf);
    display.setFont(ArialMT_Plain_10); 
    display.drawString(70,31,"µg/m  PM");
    x = display.getStringWidth("µg/m");
    display.drawString(70+x,28,"3");
    x = display.getStringWidth("µg/m  PM");
    display.drawString(70+x,33,"10");
    
    display.setFont(ArialMT_Plain_10);    
    //sprintf(puf,"PM1/2.5/10:%d/%d/%d", pm1, pm25,pm10);
    //display.drawString(1,40,puf);
    //display.setFont(ArialMT_Plain_16);

    sprintf(puf,"%d pm1, %d pm 2.5, %d %%relF", pm1, pm25, HYG);
    display.drawString(1,48,puf);
    display.display();
    
    
}

void displayChem(float NH3, float CO, float C2H5OH, float NO2) {
#ifdef PRINT_VIA_SERIAL
    Serial.print("The concentration of NH3 is [ppm]"); Serial.println(NH3);
    Serial.print("The concentration of NO2 is [ppm]"); Serial.println(NO2);
    Serial.print("The concentration of CO is [ppm]"); Serial.println(CO);
    Serial.print("The concentration of Alcohol is [ppm]"); Serial.println(C2H5OH);
#endif
   
    display.clear();
    display.setFont(ArialMT_Plain_16);
    sprintf(puf,"%4.1f ppm CO",CO);  
    display.drawString(1,0,puf);
    display.setFont(ArialMT_Plain_10);
    sprintf(puf,"(oder %4.1f ppm Alkohol)",C2H5OH);
    display.drawString(1,16,puf);
    display.drawLine(10,27,117,27);
    display.setFont(ArialMT_Plain_16);
    sprintf(puf,"%4.1f ppm NO2",NO2);
    display.drawString(1,28,puf);
    display.drawLine(10,45,117,45);
    sprintf(puf,"%4.1f ppm NH3",NH3);
    display.drawString(1,46,puf);

    display.display();
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
