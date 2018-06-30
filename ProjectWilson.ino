


/***********************************************************
 * Project Wilson: An Arduino Drift Buoy
 * Designed in San Francisco, California, USA
 * 
 * (c) 2018 Hayden Brophy and Michael Brophy
 * Except where otherwise noted, this code is licensed under Creative Commons Attribution 3.0 Unported License
 * 
 * Special thanks to:
 * Cyrille Medard de Chardon and Christophe Trefois for Humidity - Temperature sensor library
 ***********************************************************/
/**********************************/
// TODO
/**********************************
 * Add temp sensors code
 * Add photoelectric sensor code
 * Add GPS sensor code
 * Add SatModem code
 * 
 * 
 * 
 */

 
/**********************************/
// IMPORT ALL NECESSARY LIBRARIES
/**********************************/
#include <SHT1x.h>
#include <RtcDateTime.h>
#include <RtcDS1307.h>
#include <RtcDS3231.h>
#include <RtcTemperature.h>
#include <RtcUtility.h>
#include <math.h> 
#include <IridiumSBD.h> //SatComm Modem
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <Sleep_n0m1.h>
#include <OneWire.h>
#include <DallasTemperature.h> //Temp sensors
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h> //GPS
#include <TinyGPS++.h> //GPS
#include <RtcDS1307.h> //real-time clock
#include <Adafruit_MPL115A2.h> //Barometric sensor

/*****************************/
// DEFINE PIN ASSIGNMENTS
/*****************************/
// HUMIDITY & TEMP SENSOR ports (Model: SHT10)
#define dataPin 30
#define sckPin 31 //serial clock

// Used for software SPI
#define LIS3DH_CLK 13
#define LIS3DH_MISO 12
#define LIS3DH_MOSI 11
// Used for hardware & software SPI
#define LIS3DH_CS 10

#define ONE_WIRE_BUS 6

// Humid accuracy +/- 5%
// Steady accuracy between 10-80
// example at 10/90 +/- 6%, 0/100 +/- 7.5%

// Temp accuracy +/- .5 degrees celsius
// Temp error increases more as we get farther from 25 cels.
// example: @ 0/50 degrees, +/- 1.2 degrees

/*****************************/
// ALLOCATE MEMORY
/*****************************/
TinyGPSPlus gps;
SoftwareSerial gpsss(51, 50);

boolean if_run_GPS = false;

// EEPROM Address Pool
const int eelat = 0;
const int eelng = 4;

const int eemonth = 8;
const int eeday = 10;
const int eeyear = 12;

const int eehour = 14;
const int eeminute = 16;

float plat = 0.000000f;
float plng = 0.000000f;
int pmonth = 0;
int pday = 0;
int pyear = 0;
int phour = 0;
int pminute = 0;

int tempMonth, tempDay, tempYear, tempHour, tempMin;
int tempLat, tempLon;



/*****************************/
// INSTANTIATE OBJECTS
/*****************************/

// Instantiate the HUMIDITY & TEMP SENSOR object
SHT1x th_sensor(dataPin, sckPin);

// Instantiate the SLEEP object
Sleep sleep;
RtcDS1307<TwoWire> Rtc(Wire);

//Callibrated Clock Cycle 10 minute timer
unsigned long sleepTime = 603500;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Define Iridium Interface
SoftwareSerial nss(10, 9);
IridiumSBD isbd(nss, 8);

SoftwareSerial ss(4,3);

/*****************************/
// DEFINE VARS
/*****************************/
//If Already Run Routine Check
boolean if_0hr_routine = false;
boolean if_12hr_routine = false;

//RTC Variables
int rtc_year;
int rtc_month;
int rtc_day;
int rtc_hour;
int rtc_minute;
int rtc_second;


void setup()
{
  Serial.begin(115200);
  ss.begin(9600);
  Rtc.Begin();
  Serial.println("Starting up");
}

void loop()
{  
  /*****************************/
  /* Sleep Clock Routine
  /*****************************/
  
  RtcDateTime now = Rtc.GetDateTime();
  printDateTime(now);
  Serial.println();
  delay(50);
  sleep.pwrDownMode(); //set sleep mode
  sleep.sleepDelay(sleepTime); //sleep for: sleepTime
  wakeup();
}

void wakeup()
{
  /*****************************/
  /* 0 Hour Routine Check
  /*****************************/
  
  if(/*rtc_hour == 0 && */rtc_minute >= 0 && rtc_minute <= 30 && if_0hr_routine == false)
  {
    hr0_routine();
    if_0hr_routine = true;
    if_12hr_routine = false;
  }

  /*****************************/
  /* 12 Hour Routine Check
  /*****************************/

  if(/*rtc_hour == 12 && */rtc_minute > 30 && rtc_minute < 0 && if_12hr_routine == false)
  {
    hr12_routine();
    if_12hr_routine = true;
    if_0hr_routine = false;
  }
}





  /*****************************/
  /* 0 Hour Main Routine
  /*****************************/


void hr0_routine()
{
  
  Serial.println("Running 0 Hour Routine...");
  delay(100);
  GPSparse();
  
}


  /*****************************/
  /* 12 Hour Main Routine
  /*****************************/


void hr12_routine()
{
  
  Serial.println("Running 12 Hour Routine...");
  delay(100);
  GPSparse();
  
}






#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );

            rtc_year = dt.Year();
            rtc_month = dt.Month();
            rtc_day = dt.Day();
            rtc_hour = dt.Hour();
            rtc_minute = dt.Minute();
            rtc_second = dt.Second();
            
    Serial.print(datestring);
    delay(100);
}

void FetchHumid()
{
  float temp_c;
  float humid;
  
  // Read values from the sensor
  humid = th_sensor.readHumidity();
  // Since the humidity reading requires the temperature we simply
  // retrieve the reading capture from the readHumidity() call. See the lib.
  //temp_c = th_sensor.retrieveTemperatureC();
  
  // Print data
  Serial.print("Temperature: ");
  Serial.print(temp_c);
  Serial.print(", Humidity: ");
  Serial.println(humid);
}

void GPSparse()
{
  /*****************************/
  /* GPS NMEA Parse
  /*****************************/

  // Displays GPS Data every time a new sentence is correctly encoded
  while (ss.available() > 0 && if_run_GPS == false)
    if (gps.encode(ss.read()))
    {
      if (gps.date.year() == 2000)
      {
        Serial.println("Waiting...");
      }
      else
      {
        FetchGPSData();
        if_run_GPS = true;
      }
    }
}

void FetchGPSData()
{

  tempLat = gps.location.lat();
  tempLon = gps.location.lng();

  tempMonth = gps.date.month();
  tempDay = gps.date.day();
  tempYear = gps.date.year();
  tempHour = gps.time.hour();
  tempMin = gps.time.minute();

  // LatLon storing to EEPROM
  EEPROM.put(eelat, tempLat);
  EEPROM.put(eelng, tempLon);

  // Timestamp storing to EEPROM
  EEPROM.put(eemonth, tempMonth);
  EEPROM.put(eeday, tempDay);
  EEPROM.put(eeyear, tempYear);
  EEPROM.put(eehour, tempHour);
  EEPROM.put(eeminute, tempMin);

  // Fetch latlon from EEPROM
  EEPROM.get(eelat, plat);
  EEPROM.get(eelng, plng);

  // Fetch timestamp from EEPROM
  EEPROM.get(eemonth, pmonth);
  EEPROM.get(eeday, pday);
  EEPROM.get(eeyear, pyear);
  EEPROM.get(eehour, phour);
  EEPROM.get(eeminute, pminute);
 
  Serial.println(F("  ------------------------------"));
  Serial.println(F("  -------PREVIOUS LOCATION------"));
  Serial.println(F("  ------------------------------"));
  Serial.println();
  Serial.print(F("  Location: "));
 
  Serial.print(plat, 6);
  Serial.print(F(","));
  Serial.println(plng, 6);
  Serial.println();
  Serial.print(F("  Date/Time: "));
  Serial.print(pmonth);
  Serial.print(F("/"));  
  Serial.print(pday);
  Serial.print(F("/"));
  Serial.print((pyear));
  
  Serial.print(F(" "));

  if ((phour) < 10) Serial.print(F("0"));
  Serial.print(phour);
  Serial.print(F(":"));
  if (pminute < 10) Serial.print(F("0"));
  Serial.print(pminute);
  Serial.println();
  Serial.println();
}