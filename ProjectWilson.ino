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

#define TEMP1_WIRE_BUS 9  //used for dedicated temp sensor 1
#define TEMP2_WIRE_BUS 10 //used for dedicated temp sensor 2
#define IridiumSerial Serial3
#define DIAGNOSTICS true // Change this to see diagnostics

SoftwareSerial gpsSS(51, 50); //used for GPS
//SoftwareSerial nss(10, 9); //used for SatModem
//IridiumSBD isbd(nss, 8); //used for SatModem

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

OneWire oneWireTemp1(TEMP1_WIRE_BUS);
OneWire oneWireTemp2(TEMP2_WIRE_BUS);
DallasTemperature tempExtAir(&oneWireTemp1);
DallasTemperature tempWater(&oneWireTemp2);
TinyGPSPlus gps;
Adafruit_MPL115A2 barometer; //barometric sensor
IridiumSBD modem(IridiumSerial); //SatModem

/*****************************/
// ALLOCATE MEMORY
/*****************************/
// EEPROM Address Pool
const int eelat = 0;
const int eelng = 4;

const int eemonth = 8;
const int eeday = 10;
const int eeyear = 12;

const int eehour = 14;
const int eeminute = 16;

const int ee_extAirTemp = 18;
const int ee_waterTemp = 22;

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

float plat = 0.000000f;
float plng = 0.000000f;
int pmonth = 0;
int pday = 0;
int pyear = 0;
int phour = 0;
int pminute = 0;
float p_extAirTemp = 0.00f;
float p_waterTemp = 0.00f;

int tempMonth, tempDay, tempYear, tempHour, tempMin;
float tempLat = 0.000000f;
float tempLon = 0.000000f;
boolean if_run_GPS = false;
int signalQuality = -1;
int err;


void setup()
{
  Serial.begin(115200); //initiate serial comm with PC
  gpsSS.begin(9600); //initiate serial comm with GPS
  tempExtAir.begin();
  tempWater.begin();
  //initiate humidity sensor comm here
  Rtc.Begin(); //initiate serial comm with real-time clock
  barometer.begin();
  IridiumSerial.begin(19200); //initiate serial comm with SatModem

  gpsAOS();
  syncRTCTimeToGPS();
  fetchExtAirTemp();
  fetchWaterTemp();
  printOutEEPROM();
  
}

void loop()
{  
  /*****************************/
  /* Sleep Clock Routine
  /*****************************/
  
 /* RtcDateTime now = Rtc.GetDateTime();
  printDateTime(now);
  Serial.println();
  delay(50);
  sleep.pwrDownMode(); //set sleep mode
  sleep.sleepDelay(sleepTime); //sleep for: sleepTime
  wakeup();*/
}


void syncRTCTimeToGPS(){
  // Fetch GPS time from EEPROM
  EEPROM.get(eemonth, pmonth);
  EEPROM.get(eeday, pday);
  EEPROM.get(eeyear, pyear);
  EEPROM.get(eehour, phour);
  EEPROM.get(eeminute, pminute);
  int pseconds = 0;

  RtcDateTime seedTime = RtcDateTime(pyear, pmonth, pday, phour, pminute, pseconds); //set this to GPS fetched time
  Rtc.SetDateTime(seedTime);
  
}

/*
void wakeup()
{
  /*****************************/
  /* 0 Hour Routine Check
  /*****************************/
  
  //if(rtc_hour == 0 && rtc_minute >= 0 && rtc_minute <= 30 && if_0hr_routine == false)
  //{
  //  hr0_routine();
  //  if_0hr_routine = true;
  //  if_12hr_routine = false;
  //}

  /*****************************/
  /* 12 Hour Routine Check
  /*****************************/
/*
  if(rtc_hour == 12 && rtc_minute > 30 && rtc_minute < 0 && if_12hr_routine == false)
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
  gpsAOS();
  
}


  /*****************************/
  /* 12 Hour Main Routine
  /*****************************/


void hr12_routine()
{
  
  Serial.println("Running 12 Hour Routine...");
  delay(100);
  gpsAOS();
  
}






#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u|%02u:%02u:%02u"),
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
//  humid = th_sensor.readHumidity();
  // Since the humidity reading requires the temperature we simply
  // retrieve the reading capture from the readHumidity() call. See the lib.
  //temp_c = th_sensor.retrieveTemperatureC();
  
  // Print data
  Serial.print("Temperature: ");
  Serial.print(temp_c);
  Serial.print(", Humidity: ");
  Serial.println(humid);
}

void gpsAOS()
{
  /*****************************/
  /* GPS NMEA Parse
  /*****************************/

  // Displays GPS Data every time a new sentence is correctly encoded
  // Loop through this until successful acquisition of signal (AOS) from GPS so we know we have data
  wait:
  while (gpsSS.available() > 0)
  {
    if (gps.encode(gpsSS.read()))
    {
      if (gps.date.year() != 2000)
      {
        gpsStoreData();
        return;
      }
    }
  }
  goto wait;
}

void gpsStoreData()
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

}

void fetchExtAirTemp(){
  float temp_extAirTemp;
  tempExtAir.requestTemperatures();
  temp_extAirTemp = tempExtAir.getTempCByIndex(0);

  // Storing to EEPROM
  EEPROM.put(ee_extAirTemp, temp_extAirTemp);
}

void fetchWaterTemp(){
  float temp_waterTemp;
  tempWater.requestTemperatures();
  temp_waterTemp = tempWater.getTempCByIndex(0);

  // Storing to EEPROM
  EEPROM.put(ee_waterTemp, temp_waterTemp);
}


void printOutEEPROM(){
  // Fetch latlon from EEPROM
  RtcDateTime now = Rtc.GetDateTime();
  printDateTime(now);
  
  EEPROM.get(eelat, plat);
  EEPROM.get(eelng, plng);

  EEPROM.get(eemonth, pmonth);
  EEPROM.get(eeday, pday);
  EEPROM.get(eeyear, pyear);
  EEPROM.get(eehour, phour);
  EEPROM.get(eeminute, pminute);

  EEPROM.get(ee_extAirTemp, p_extAirTemp);
  EEPROM.get(ee_waterTemp, p_waterTemp);
  
  Serial.print("|");
  Serial.print(plat, 6);
  Serial.print("|");
  Serial.print(plng, 6);
  Serial.print("|");
  Serial.print(p_extAirTemp, 1);
  Serial.print("|");
  Serial.print(p_waterTemp, 1);
  Serial.println();
}




// Humid accuracy +/- 5%
// Steady accuracy between 10-80
// example at 10/90 +/- 6%, 0/100 +/- 7.5%

// Temp accuracy +/- .5 degrees celsius
// Temp error increases more as we get farther from 25 cels.
// example: @ 0/50 degrees, +/- 1.2 degrees
