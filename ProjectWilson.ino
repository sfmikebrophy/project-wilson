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
 * Add photoelectric sensor code
 * Add SatModem code
 * Add Data Package code
 * Add Fault Tolerance code
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

SoftwareSerial gpsSS(11, 50); //used for GPS
//SoftwareSerial nss(10, 9); //used for SatModem
//IridiumSBD isbd(nss, 8); //used for SatModem

/*****************************/
// INSTANTIATE OBJECTS
/*****************************/

// Instantiate the HUMIDITY & TEMP SENSOR object
SHT1x sht1x(dataPin, sckPin);

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
const int ee_intAirTemp = 26;

const int ee_humidity = 30;

const int ee_barometric = 34;

const int ee_lightIntensity = 38;
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
float p_intAirTemp = 0.00f;
float cTemp = 0.00f;
float p_humidity = 0.00f;
float p_barometric = 0.00f;
int p_lightIntensity = 0;
int lightSensorPin = A1;    // select the input pin for the photosensor

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
  //IridiumSerial.begin(19200); //initiate serial comm with SatModem

  Serial.println("Starting up...");
  delay(100);
  
  //SYSTEMS TESTING
  gpsAOS();
  syncRTCTimeToGPS();
  fetchExtAirTemp();
  fetchWaterTemp();
  fetchIntAirTemp();
  fetchHumidity();
  fetchBarometric();
  fetchLightSensorData();
  printOutEEPROM();
}

void loop()
{  
  /*****************************/
  /* Sleep Clock Routine
  /*****************************/

/*

  Serial.println("Going to Sleep...");
  RtcDateTime now = Rtc.GetDateTime();
  printDateTime(now);
  Serial.println();
  delay(50);
  sleep.pwrDownMode(); //set sleep mode
  sleep.sleepDelay(sleepTime); //sleep for: sleepTime
  Serial.println("Waking up...");
  wakeup();

*/
  
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

  if(/*rtc_hour == 12 && */rtc_minute >= 0 && rtc_minute <= 30 && if_12hr_routine == false)
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
  Serial.println();
  delay(100);
  gpsAOS();
  syncRTCTimeToGPS();
  fetchExtAirTemp();
  fetchWaterTemp();
  fetchIntAirTemp();
  fetchHumidity();
  fetchBarometric();
  printOutEEPROM();
  
}


  /*****************************/
  /* 12 Hour Main Routine
  /*****************************/


void hr12_routine()
{
  
  Serial.println("Running 12 Hour Routine...");
  Serial.println();
  delay(100);
  gpsAOS();
  syncRTCTimeToGPS();
  fetchExtAirTemp();
  fetchWaterTemp();
  fetchIntAirTemp();
  fetchHumidity();
  fetchBarometric();
  printOutEEPROM();
  
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
  temp_extAirTemp = convertTemp(temp_extAirTemp) - 4.7;

  // Storing to EEPROM
  EEPROM.put(ee_extAirTemp, temp_extAirTemp);
}

void fetchWaterTemp(){
  float temp_waterTemp;
  tempWater.requestTemperatures();
  temp_waterTemp = tempWater.getTempCByIndex(0);
  temp_waterTemp = convertTemp(temp_waterTemp);

  // Storing to EEPROM
  EEPROM.put(ee_waterTemp, temp_waterTemp);
}

void fetchIntAirTemp()
{
  float temp_null;
  float temp_intAirTemp;

  // Read values from sensor
  barometer.getPT(&temp_null,&temp_intAirTemp);
  
  // Convert values into F 
  temp_intAirTemp = convertTemp(temp_intAirTemp);

  // Storing to EEPROM
  EEPROM.put(ee_intAirTemp, temp_intAirTemp);
}

void fetchHumidity()
{
  float temp_humidity;

  // Read values from the sensor
  temp_humidity = sht1x.readHumidity();

  // Storing to EEPROM
  EEPROM.put(ee_humidity, temp_humidity);
}

void fetchBarometric()
{
  float temp_pressureKPA;
  float temp_pressure_inHG;
  
  // Read values from the sensor
  temp_pressureKPA = barometer.getPressure();

  // Convery values into inHG
  temp_pressure_inHG = temp_pressureKPA * 0.295301;

  // Storing to EEPROM
  EEPROM.put(ee_barometric, temp_pressure_inHG);
}

void fetchLightSensorData()
{
  // read the value from the photosensor
  p_lightIntensity = analogRead(lightSensorPin);
  Serial.print(p_lightIntensity);
  Serial.println();
  // Storing to EEPROM
  EEPROM.put(ee_lightIntensity, p_lightIntensity);
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
  EEPROM.get(ee_humidity, p_humidity);
  EEPROM.get(ee_intAirTemp, p_intAirTemp);
  EEPROM.get(ee_barometric, p_barometric);
  EEPROM.get(ee_lightIntensity, p_lightIntensity);
  
  Serial.print("|");
  Serial.print(plat, 6);
  Serial.print("|");
  Serial.print(plng, 6);
  Serial.print("|");
  Serial.print(p_extAirTemp, 1);
  Serial.print("|");
  Serial.print(p_waterTemp, 1);
  Serial.print("|");
  Serial.print(p_intAirTemp, 1);
  Serial.print("|");
  Serial.print(p_humidity, 1);
  Serial.print("|");
  Serial.print(p_barometric, 2);
  Serial.print("|");
  Serial.print(p_lightIntensity, 1);
  Serial.println();
  Serial.println();
}

float convertTemp(float cTemp){
  float fTemp = (cTemp * 1.8) + 32;
  return fTemp;
}




// Humid accuracy +/- 5%
// Steady accuracy between 10-80
// example at 10/90 +/- 6%, 0/100 +/- 7.5%

// Temp accuracy +/- .5 degrees celsius
// Temp error increases more as we get farther from 25 cels.
// example: @ 0/50 degrees, +/- 1.2 degrees
