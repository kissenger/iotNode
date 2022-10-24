
/*
 * Error codes (indicated by number of LED flashes):
 *  0  - No error - "200 OK" response from server
 *  1  - Cannot establish a connection to host server
 *  2  - Server connected, but no response after GET or POST
 *  3  - "404 Not Found" response from server
 *  4  - "502 Bad Gateway" response from server
 *  5 -  "500 Internal Server Error" response from server (rejected promise from mongo)
 *  6  - Other response returned from server (200, 404, 502 or 500 not found in return string)
 *  8  - A defined sensor was unexpectedly not found during setup
 *  SOLID LED - Not connected to Wifi
 */

/* 
 * -----------------------------------------------------------------------------------------------------*  
 * INTERRUPTS
 * Everything in this block is defining blink behavious in case of error
 * Interrupt library here: https://github.com/khoih-prog/SAMD_TimerInterrupt
 * -----------------------------------------------------------------------------------------------------* 
 */
 
#define TIMER_INTERRUPT_DEBUG         0          // per the documentation, these #defines need to be above the #includes
#define _TIMERINTERRUPT_LOGLEVEL_     0
#define USING_TIMER_TCC1               true

#include "SAMDTimerInterrupt.h"
#include "SAMD_ISR_Timer.h"

#define HW_TIMER_INTERVAL_MS          10L
#define TIMER_INTERVAL                250L    
#define GAP_BTWN_BLINKS               1
#define GAP_AFTER_BLINKS              10

SAMDTimer ITimer1(TIMER_TCC1);
SAMD_ISR_Timer ISR_Timer_TCC1;

volatile uint8_t interruptCounter = 99;
volatile uint8_t blinkCounter     = 0;
volatile uint8_t errorCode        = 0;        // this is the only variable of interest to the main loop 

void errorBlinker() {

  if (errorCode) {
    interruptCounter++;
    if (blinkCounter < errorCode) {
      if (interruptCounter > GAP_BTWN_BLINKS) {
  //        Serial.print("H");                            // cannot use DEBUG_PRINT as not yet defined
        digitalWrite(LED_BUILTIN, HIGH);
        blinkCounter++;
        interruptCounter = 0;
      } else {
  //        Serial.print("L");
        digitalWrite(LED_BUILTIN, LOW);
      }
    } else {  //blinkCounter == errorCode
  //      Serial.print("P");
      digitalWrite(LED_BUILTIN, LOW);
      if (interruptCounter == GAP_AFTER_BLINKS) {
        blinkCounter = 0;
        interruptCounter = 99;
      }
    }
  }
  
}

void TimerHandler_TCC1(void) {
  ISR_Timer_TCC1.run();
}

/*
 * ----------------------------------------------------------------------------------------------------- 
 * INITIALISATION 
 * No user intervention needed in this section
 * -----------------------------------------------------------------------------------------------------
 */

#include <WiFiNINA.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_SHT31.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <OneWire.h>            // library for dallas temp gauges
#include <DallasTemperature.h>  // library for dallas temp gauges

#define WIFI_PWD                  "qwertyuiopisthetoprowofkeysonakeyboard"
//#define BMPADDR                   0x77
#define MUXADDR                   0x70
#define SHTADDR                   0x44
#define MODE_DEPLOYED_GARAGE      0
#define MODE_DEPLOYED_HOUSE       1
#define MODE_TEST_LIVE            2
#define MODE_TEST_LOCAL           3
#define HTML_CODE_200             "200 OK"
#define HTML_CODE_400             "400 Bad Request" 
#define HTML_CODE_404             "404 Not Found" 
#define HTML_CODE_502             "502 Bad Gateway"
#define HTML_CODE_400             "500 Internal Server Error"
#define PIN_DIGITAL_TEMPS         2
#define SERVER_RESPONSE_TIMEOUT   20000
#define WIFI_LOOP_DELAY           10000
#define SHT_HEATER_ON_TIME        5000
#define SHT_HEATER_OFF_TIME       100000        //ms
#define SHT_HEATER_PREHEAT_THRESH 90

WiFiClient        client;
OneWire           oneWire(PIN_DIGITAL_TEMPS);
DallasTemperature DallasSensors(&oneWire);
uint32_t          lastSensorTime           = 0; 
uint32_t          interval;
bool              forceSensorLoop          = true;
bool              responseRecieved         = true;
bool              preHeatConditionsChecked = false;
char *            serverResponse           = "";



typedef struct{
  char * sensorName;
  DeviceAddress address;
  bool isFound;
} dallasSensor;


typedef struct{
  char * sensorName;
  uint8_t muxChannel;
  bool isFound;
} ahtSensor;

typedef struct{
  char * sensorName;
  uint8_t muxChannel;
  bool isFound;
} bmpSensor;

typedef struct{
  char * sensorName;
  uint8_t muxChannel;
  bool isFound;
} shtSensor;

/*
 * ----------------------------------------------------------------------------------------------------- 
 * DEFINE SENSORS AND RUN_MODE
 * All user defined parameters in this section
 * -----------------------------------------------------------------------------------------------------
 */

//#define RUN_MODE                  MODE_DEPLOYED_GARAGE
#define RUN_MODE                  MODE_DEPLOYED_HOUSE
//#define RUN_MODE                  MODE_TEST_LOCAL
//#define RUN_MODE                  MODE_TEST_LIVE

dallasSensor dallasSensors[] = {    //   sensorName, I2C address, isFound
//  { "Hall",             {0x28, 0x95, 0x1F, 0xE4, 0xB1, 0x21, 0x06, 0xDF},     false },
//  { "Hall Radiator",    {0x28, 0xAD, 0xB0, 0x08, 0x0A, 0x00, 0x00, 0x31},     false },
//  { "Living Room",      {0x28, 0x13, 0x9E, 0x0A, 0x0A, 0x00, 0x00, 0x78},     false },
//  { "Kitchen",          {0x28, 0xF0, 0x2E, 0x7C, 0x05, 0x00, 0x00, 0xB9},     false }
//  { "Gordons Office",         {0x28, 0x8D, 0xCB, 0x00, 0x04, 0x00, 0x00, 0x72},     false }
//    { "dallasOutside",          {0x28, 0xB0, 0x14, 0x81, 0xE3, 0x31, 0x3C, 0xFB}, false},
//    { "dallasStoreRoom",        {0x28, 0x29, 0x09, 0x7C, 0x05, 0x00, 0x00, 0x42}, false}
    { "dallasTestOnly",        {0x28, 0xB1, 0xB3, 0x9E, 0xB1, 0x21, 0x06, 0x14}, false}
};

ahtSensor ahtSensors[] = {        //  sensorName, muxChannel, isFound
  // use muxChannel = 99 if mux is not in use
//  { "ahtInside",      0,       false},
//  { "ahtStoreRoom",     99,       false},
//  { "ahtTest",        2,       false}
};

bmpSensor bmpSensors[] = {       //  sensorName, muxChannel, isFound
//  { "bmpStoreRoom",  99 , false}
};

shtSensor shtSensors[] = {       //  sensorName, muxChannel, isFound
  { "shtOutside",  99 , false}
};

/*
 * -----------------------------------------------------------------------------------------------------
 * FINAL SETUP
 * Initialisation based on run mode and defined sensors
 * -----------------------------------------------------------------------------------------------------
 */

#if RUN_MODE == MODE_TEST_LIVE || RUN_MODE == MODE_TEST_LOCAL
  #define DEBUG_PRINT(x)   Serial.print(x);
  #define DEBUG_PRINTLN(x) Serial.println(x);
  #define DEBUG_WRITE(x)   Serial.write(x);
#else
  #define DEBUG_PRINT(x);
  #define DEBUG_PRINTLN(x);
  #define DEBUG_WRITE(x);
#endif

#if RUN_MODE == MODE_TEST_LIVE
  #define         IS_DEPLOYED      false
  #define         WIFI_NAME        "IvyTerrace"
  #define         SERVER_PATH      "/iot/api/new-data"
  #define         SERVER_PORT      80
  #define         READ_INTERVAL    10
//  #define         READ_INTERVAL    1
  const char      SERVER_HOST[]    = "www.thingummy.cc";  
  
#elif RUN_MODE == MODE_TEST_LOCAL 
  #define         IS_DEPLOYED      false
  #define         WIFI_NAME        "IvyTerrace"
  #define         SERVER_PATH      "/api/new-data"
//  #define         SERVER_PATH      "/api/ping"
  #define         SERVER_PORT      3000
  #define         READ_INTERVAL    0.1
  const IPAddress SERVER_HOST(192,168,1,64);
    
#elif RUN_MODE == MODE_DEPLOYED_GARAGE
  #define         IS_DEPLOYED      true
  #define         WIFI_NAME        "IvyTerrace_EXT"
  #define         SERVER_PATH      "/iot/api/new-data"
  #define         SERVER_PORT      80
  #define         READ_INTERVAL    15
  const char      SERVER_HOST[]    = "www.thingummy.cc";  

#elif RUN_MODE == MODE_DEPLOYED_HOUSE
  #define         IS_DEPLOYED      true
  #define         WIFI_NAME        "IvyTerrace"
  #define         SERVER_PATH      "/iot/api/new-data"
  #define         SERVER_PORT      80
  #define         READ_INTERVAL    15
  const char      SERVER_HOST[]    = "www.thingummy.cc";
    
#endif

const uint32_t    readSensorInterval      = READ_INTERVAL * 60UL * 1000UL;
const uint8_t     nDallasSensors            = sizeof(dallasSensors) / sizeof(dallasSensors[0]);
const uint8_t     nAhtSensors               = sizeof(ahtSensors) / sizeof(ahtSensors[0]);
const uint8_t     nBmpSensors               = sizeof(bmpSensors) / sizeof(bmpSensors[0]);
const uint8_t     nShtSensors               = sizeof(shtSensors) / sizeof(shtSensors[0]);
uint32_t          shtHeaterOnTime[nShtSensors];
uint32_t          shtHeaterOffTime[nShtSensors];
bool              shtIsHeaterOn[nShtSensors];
//float             shtPrecheckTemp[nShtSensors];
//float             shtPrecheckRH[nShtSensors]; 

Adafruit_AHTX0  ahtInstance[nAhtSensors]; 
Adafruit_BMP085 bmpInstance[nBmpSensors];
Adafruit_SHT31  shtInstance[nShtSensors];
sensors_event_t ahtHumidity, ahtTemp;





/*
 * -----------------------------------------------------------------------------------------------------
 * SETUP LOOP
 * ----------------------------------------------------------------------------------------------------- 
 */

void setup() {


  #if RUN_MODE == MODE_TEST_LIVE || RUN_MODE == MODE_TEST_LOCAL 
    Serial.begin(9600);
    while (!Serial);
    DEBUG_PRINTLN("Serial Connected");
  #endif
  
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_DIGITAL_TEMPS, INPUT);

  // Initialise DS18B20 temperature sensors
  DallasSensors.begin();  
  DallasSensors.requestTemperatures();
  for (uint8_t i = 0; i < nDallasSensors; i++) {
    if ( DallasSensors.getTempC(dallasSensors[i].address) > -100 ) {
      dallasSensors[i].isFound = true;
      DallasSensors.setResolution(dallasSensors[i].address, 12);
    } else {
      DEBUG_PRINTLN("WARNING: Could not find sensor \"" +  String(dallasSensors[i].sensorName) + "\"");
      errorCode = 8;  
    }
  }

  // Initialise AHT humidity sensors
  for (uint8_t i = 0; i < nAhtSensors; i++) {
    if ( ahtSensors[i].muxChannel != 99) {
      selectMuxChannel(ahtSensors[i].muxChannel);
    }
    if ( ahtInstance[i].begin() ) {
      ahtSensors[i].isFound = true;
    } else {
      DEBUG_PRINTLN("WARNING: Could not find sensor \""  + String(ahtSensors[i].sensorName) + "\"");
      errorCode = 8;      
    }
  }

  // Initialise BMP pressure sensors
  for (uint8_t i = 0; i < nBmpSensors; i++) {
    if ( ahtSensors[i].muxChannel != 99) {
      selectMuxChannel(bmpSensors[i].muxChannel);
    }
    if ( bmpInstance[i].begin() ) {
      bmpSensors[i].isFound = true;
    } else {
      DEBUG_PRINTLN("WARNING: Could not find sensor \""  + String(bmpSensors[i].sensorName) + "\"");
      errorCode = 8;
    }
  }

  // Initialise SHT humidity sensors
  for (uint8_t i = 0; i < nShtSensors; i++) {
    shtHeaterOnTime[i] = 0;
    shtHeaterOffTime[i] = 0;
    shtIsHeaterOn[i] = false;    
    if ( shtSensors[i].muxChannel != 99) {
      selectMuxChannel( shtSensors[i].muxChannel );
    }
    if ( shtInstance[i].begin(SHTADDR) ) {
      shtSensors[i].isFound = true;
    } else {
      DEBUG_PRINTLN("WARNING: Could not find sensor \""  + String(shtSensors[i].sensorName) + "\"");
      errorCode = 8;
    }
  }  
  
  // Start blink timer
  ITimer1.attachInterruptInterval_MS(HW_TIMER_INTERVAL_MS, TimerHandler_TCC1);
  ISR_Timer_TCC1.setInterval(TIMER_INTERVAL, errorBlinker);
}

/*
 * -----------------------------------------------------------------------------------------------------
 * MAIN LOOP
 * -----------------------------------------------------------------------------------------------------
 */

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();
  }

  // check for server response
  // listenForResponse flag means we check every line until something 
  if ( serverResponse == "" && client.available() ) {
    serverResponse = getServerResponse();
    responseRecieved = true;
    if ( strstr(serverResponse, HTML_CODE_200) ) {
      DEBUG_PRINTLN("Code 200 received, setting errorCode to 0");
      errorCode = 0;    
    } else if ( strstr(serverResponse, HTML_CODE_404) ) {
      DEBUG_PRINTLN("Code 404 received, setting errorCode to 3");
      errorCode = 3;
    } else if ( strstr(serverResponse, HTML_CODE_502) ) {
      DEBUG_PRINTLN("Code 502 received, setting errorCode to 4");
      errorCode = 4;    
    } else if ( strstr(serverResponse, HTML_CODE_400) ) {
      DEBUG_PRINTLN("Code 500 received, setting errorCode to 5");
      errorCode = 5;
    } else { // this line contains something we werent expecting, set error code and check again next loop
      DEBUG_PRINTLN("Unexpected server response, setting errorCode to 6");
      errorCode = 6;
      serverResponse = "";
    }
  } 

  // if no response received from server within timeout, set error
  if ( !responseRecieved  && millis() - lastSensorTime > SERVER_RESPONSE_TIMEOUT ) {
    DEBUG_PRINTLN("No server response after timeout, setting errorCode to 2");
    if ( errorCode != 1 && errorCode != 2 ) {
      errorCode = 2;
    }
  }

  // force sensor loop at reduced interval if there is an error code, or if heater is enabled (on)
//  if ( errorCode ) {
//    interval = SERVER_RESPONSE_TIMEOUT;
//  } else {
//    interval = readSensorInterval;
//  }

  //turn heater off if its been on for longet than SHT_HEATER_ON_TIME
  for (uint8_t i = 0; i < nShtSensors; i++ ) {
    if ( millis() - shtHeaterOnTime[i] > SHT_HEATER_ON_TIME ) {
      if ( shtHeaterOnTime[i] >= shtHeaterOffTime[i] ) {  // heater is off    
        DEBUG_PRINTLN("SHT " + String(i) + ": Preheater off.");      
        shtHeaterOffTime[i] = shtTurnOffHeater(i);
      }
    }
  }

  for (uint8_t i = 0; i < nShtSensors; i++ ) {
    if ( 
      millis() - shtHeaterOffTime[i] > SHT_HEATER_OFF_TIME &&                 // heater has been off long enough
      millis() - lastSensorTime < readSensorInterval - SHT_HEATER_ON_TIME - SHT_HEATER_OFF_TIME    // there is enough time to turn heater on and off before taking measurement
      ) {
        if ( shtHeaterOnTime[i] <= shtHeaterOffTime[i] ) {  // heater is off        
          if ( shtInstance[i].readHumidity() > SHT_HEATER_PREHEAT_THRESH ) {
            DEBUG_PRINTLN("SHT " + String(i) + ": High RH before measurement, preheater on.");
            shtHeaterOnTime[i] = shtTurnOnHeater(i);
          }
        }         
      }
  }
  
  
  // sensor loop
  if ( millis() - lastSensorTime > readSensorInterval || forceSensorLoop ) {

    // set up loop variables
    preHeatConditionsChecked = false;
    lastSensorTime = millis();
    forceSensorLoop = false;
    responseRecieved = false;
    serverResponse = "";
    DynamicJsonDocument doc(2048);
    JsonArray data = doc.createNestedArray("data");
    
    // get temperature data from dallas sensors bus
    if ( nDallasSensors > 0) {
      DallasSensors.requestTemperatures();
    }      
    for (uint8_t i = 0; i < nDallasSensors; i++) {
      if ( dallasSensors[i].isFound ) {
        JsonObject obj = data.createNestedObject();
        obj["sensor_name"] = dallasSensors[i].sensorName;
        obj["sensor_type"] = "ds18b20";
        obj["deployed"] = IS_DEPLOYED; 
        obj["temp"] = DallasSensors.getTempC(dallasSensors[i].address);
      }
    }


    // get data from aht sensors 
    for (uint8_t i = 0; i < nAhtSensors; i++) {
      if ( ahtSensors[i].isFound ) {
        JsonObject obj = data.createNestedObject();
        if ( ahtSensors[i].muxChannel != 99) {
          selectMuxChannel(ahtSensors[i].muxChannel);
        }
        ahtInstance[i].getEvent(&ahtHumidity, &ahtTemp);
        obj["sensor_name"] = ahtSensors[i].sensorName;
        obj["sensor_type"] = "ahtx0";
        obj["deployed"] = IS_DEPLOYED;
        obj["temp"] = ahtTemp.temperature;
        obj["rh"] = ahtHumidity.relative_humidity;
      }
    }   

    // get data from bmp sensors
    for (uint8_t i = 0; i < nBmpSensors; i++) {
      if ( bmpSensors[i].isFound ) {
        JsonObject obj = data.createNestedObject();
        if ( ahtSensors[i].muxChannel != 99) {
          selectMuxChannel(bmpSensors[i].muxChannel);
        }
        obj["sensor_name"] = bmpSensors[i].sensorName;
        obj["sensor_type"] = "bmp180";
        obj["deployed"] = IS_DEPLOYED;
        obj["press"] = bmpInstance[i].readPressure();
        obj["temp"] = bmpInstance[i].readTemperature();
      }
    }   

    // get data from sht sensors
    for (uint8_t i = 0; i < nShtSensors; i++) {
      if ( shtSensors[i].isFound ) {
        JsonObject obj = data.createNestedObject();
        if ( shtSensors[i].muxChannel != 99) {
          selectMuxChannel( shtSensors[i].muxChannel );
        }
        obj["sensor_name"] = shtSensors[i].sensorName;
        obj["sensor_type"] = "sht31";
        obj["deployed"] = IS_DEPLOYED;
        obj["temp"] = shtInstance[i].readTemperature();
        obj["rh"] = shtInstance[i].readHumidity();
//        obj["temp_pre"] = shtPrecheckTemp[i];
//        obj["rh_pre"] = shtPrecheckRH[i];     
//        obj["heater_on"] = shtHeaterOnTime[i] > shtHeaterOffTime[i];
      }
      // leave heater turnoff until main loop
    }
    
    // Check for server connection 
    client.stop();
    if (client.connect(SERVER_HOST, SERVER_PORT)) {
      errorCode = errorCode == 1 ? 0 : errorCode;   // only reset errorCode if it was 1 to start with
    } else {
      DEBUG_PRINTLN("Cannot connect to server, setting errorCode to 1");
      errorCode = 1;
    }

    // Send data via POST request
    static char jsonBuffer[2048];                 // this approach from here to avoide clipped output: https://s-gregorini003.github.io/post/large-json-data/
//    serializeJsonPretty(doc, jsonBuffer); 
    serializeJson(doc, jsonBuffer);     
    DEBUG_PRINTLN(jsonBuffer);
    postRequest(String(jsonBuffer));

    // Ping server using GET request (debugging)
//    DEBUG_PRINTLN("Ping server via GET request");
//    getRequest();

    DEBUG_PRINTLN("End of sensor loop, errorCode = " + String(errorCode));
      
  }
}

/*
 * -----------------------------------------------------------------------------------------------------
 * FUNCTIONS
 * -----------------------------------------------------------------------------------------------------
 */

// function connectWifi()
// Blocks until wifi is connected, printing to screen and showing solid LED 
void connectWifi() {
  DEBUG_PRINT("Connecting to " + String(WIFI_NAME));
  digitalWrite(LED_BUILTIN, HIGH);
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_NAME, WIFI_PWD);
    DEBUG_PRINT(".");
    delay(WIFI_LOOP_DELAY);
  }
  DEBUG_PRINTLN("...OK");
  digitalWrite(LED_BUILTIN, LOW);
}


// function getServerResponse
// Returns only the first line of server response
char * getServerResponse() {
  static char firstLine[50];
  uint8_t pos = 0;
  while (client.available()) {
    char inByte = client.read();
    DEBUG_WRITE(inByte);
    if (inByte == '\n') {
//      if ( pos > 5 ) {      // in case \n is the first character, ignore it - needed for live server
        break;
//      }
    }
    firstLine[pos++] = inByte;
  }
  return firstLine;
}

void postRequest(String body) {
  // send HTTP request header
  // https://stackoverflow.com/questions/58136179/post-request-with-wifinina-library-on-arduino-uno-wifi-rev-2
  client.println("POST " + String(SERVER_PATH) + " HTTP/1.1");
  client.println("Host: " + String(SERVER_HOST));
  client.println("Content-Type: application/json");
  client.println("Accept: */*");
  client.println("Cache-Control: no-cache");
  client.println("Accept-Encoding: gzip, deflate");
  client.println("Accept-Language: en-us");
  client.println("Content-Length: " + String(body.length()));
  client.println("Connection: close");
  client.println(); // end HTTP request header
  client.println(body);
}

void getRequest() {
  // send HTTP request header
  // https://stackoverflow.com/questions/58136179/post-request-with-wifinina-library-on-arduino-uno-wifi-rev-2
  client.println("GET " + String(SERVER_PATH) + " HTTP/1.1");
  client.println("Host: " + String(SERVER_HOST));
  client.println("Connection: close");
  client.println(); // end HTTP request header
  client.println();
}

void selectMuxChannel(int i) {
  if (i > 7) return;
  Wire.beginTransmission(MUXADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

uint32_t shtTurnOnHeater(uint8_t index) {
  shtInstance[index].heater(true); 
  return millis();
}

uint32_t shtTurnOffHeater(uint8_t index) {
  shtInstance[index].heater(false); 
  return millis();
}

//float fTo2dp(float n) {
//   return (int)(n * 100 + 0.5) / 100.0;
//}
