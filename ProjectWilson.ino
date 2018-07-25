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
#include <RTClib.h> 
#include <IridiumSBD.h> //SatComm Modem
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <DallasTemperature.h> //Temp sensors
#include <Wire.h>
#include <SPI.h>
#include <TinyGPS.h> //GPS
#include <TinyGPS++.h> //GPS
#include <Adafruit_MPL115A2.h> //Barometric sensor
#include <avr/wdt.h>            // library for default watchdog functions
#include <avr/interrupt.h>      // library for interrupts handling
#include <avr/sleep.h>          // library for sleep
#include <avr/power.h>          // library for power control

/*****************************/
// DEFINE PIN ASSIGNMENTS
/*****************************/
// HUMIDITY & TEMP SENSOR ports (Model: SHT10)
#define dataPin 30
#define sckPin 31 //serial clock

#define TEMP1_WIRE_BUS 9  //used for dedicated temp sensor 1
#define TEMP2_WIRE_BUS 10 //used for dedicated temp sensor 2
#define IridiumSerial Serial
#define DIAGNOSTICS true // Change this to see diagnostics

SoftwareSerial gpsSS(11, 50); //used for GPS
//SoftwareSerial nss(10, 9); //used for SatModem
//IridiumSBD isbd(nss, 8); //used for SatModem

/*****************************/
// INSTANTIATE OBJECTS
/*****************************/

// Instantiate the HUMIDITY & TEMP SENSOR object
SHT1x sht1x(dataPin, sckPin);

RTC_PCF8523 rtc; //instatiate RTC

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
// how many times remain to sleep before wake up
// volatile to be modified in interrupt function
volatile int nbr_remaining; 

// pin on which a led is attached on the board
#define led 13

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
  Serial.println("In setup()...");
  delay(100);
  
  //watchdog and sleep functionality
  // use led 13 and put it in low mode
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  
  delay(1000);
  
  // configure the watchdog
  configure_wdt();

  Serial.begin(115200); //initiate serial comm with PC
  gpsSS.begin(9600); //initiate serial comm with GPS
  tempExtAir.begin();
  tempWater.begin();
  //initiate humidity sensor comm here
  rtc.begin(); //initiate serial comm with real-time clock
  Serial.println("Initiating RTC...");
  delay(100);
  
  //gpsAOS(); //acquire GPS signal and time
  //syncRTCTimeToGPS(); //sync RTC to time from GPS
  rtc.adjust(DateTime(2018, 7, 24, 5, 20, 0)); //for testing only
  Serial.println("Setting RTC clock...");
  delay(500);
  
  barometer.begin();
  //IridiumSerial.begin(19200); //initiate serial comm with SatModem

  Serial.println("Exiting setup()...");
  delay(100);

}

void loop()
{  
  // sleep for a given number of cycles in lowest power mode, each cycle = 8 secs
  sleep(1); //run every 10 minutes, which is 75 cycles
  wakeup(); //time check

  // testing only
  /*
  gpsAOS();
  syncRTCTimeToGPS();
  fetchExtAirTemp();
  fetchWaterTemp();
  fetchIntAirTemp();
  fetchHumidity();
  fetchBarometric();
  fetchLightSensorData();
  printOutEEPROM();
  */
  
  // now a real hang is happening: this will rebood the board
  // after 8 seconds
  // (at the end of the sleep, nrb_remaining = 0)
  //while (1); //use for hang testing only
  
}


void syncRTCTimeToGPS(){
  // Fetch GPS time from EEPROM
  EEPROM.get(eemonth, pmonth);
  EEPROM.get(eeday, pday);
  EEPROM.get(eeyear, pyear);
  EEPROM.get(eehour, phour);
  EEPROM.get(eeminute, pminute);
  int pseconds = 0;

  rtc.adjust(DateTime(pyear, pmonth, pday, phour, pminute, pseconds));
}


void wakeup()
{
  /*****************************/
  /* 0 Hour Routine Check
  /*****************************/
  DateTime now = rtc.now();
  printDateTime(now);
  Serial.println();
  delay(100);
  if(now.hour() == 5 && if_0hr_routine == false)
  {
    hr0_routine();
    if_0hr_routine = true;
    if_12hr_routine = false;
  }

  /*****************************/
  /* 12 Hour Routine Check
  /*****************************/

  if(now.hour() == 12 && if_12hr_routine == false)
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
  fetchLightSensorData();
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
  fetchLightSensorData();
  printOutEEPROM();
  
}






#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const DateTime& dt)
{
    DateTime now = rtc.now();
    
    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
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

  // Storing to EEPROM
  EEPROM.put(ee_lightIntensity, p_lightIntensity);
}

void printOutEEPROM(){
  // Fetch latlon from EEPROM
  
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


// interrupt raised by the watchdog firing
// when the watchdog fires, this function will be executed
// remember that interrupts are disabled in ISR functions
// now we must check if the board is sleeping or if this is a genuine
// interrupt (hanging)
ISR(WDT_vect)
{
    // Check if we are in sleep mode or it is a genuine WDR.
    if(nbr_remaining > 0)
    {
        // not hang out, just waiting
        // decrease the number of remaining avail loops
        nbr_remaining = nbr_remaining - 1;
        wdt_reset();
    }
    else
    {
        // must be rebooted
        // configure
        MCUSR = 0;                          // reset flags
       
                                            // Put timer in reset-only mode:
        WDTCSR |= 0b00011000;               // Enter config mode.
        WDTCSR =  0b00001000 | 0b000000;    // clr WDIE (interrupt enable...7th from left)
                                            // set WDE (reset enable...4th from left), and set delay interval
                                            // reset system in 16 ms...
                                            // unless wdt_disable() in loop() is reached first
                                       
        // reboot
        while(1);
    }
}


// function to configure the watchdog: let it sleep 8 seconds before firing
// when firing, configure it for resuming program execution by default
// hangs will correspond to watchdog fired when nbr_remaining <= 0 and will
// be determined in the ISR function
void configure_wdt(void)
{
 
  cli();                           // disable interrupts for changing the registers

  MCUSR = 0;                       // reset status register flags

                                   // Put timer in interrupt-only mode:                                       
  WDTCSR |= 0b00011000;            // Set WDCE (5th from left) and WDE (4th from left) to enter config mode,
                                   // using bitwise OR assignment (leaves other bits unchanged).
  WDTCSR =  0b01000000 | 0b100001; // set WDIE: interrupt enabled
                                   // clr WDE: reset disabled
                                   // and set delay interval (right side of bar) to 8 seconds

  sei();                           // re-enable interrupts 
}

// Put the Arduino to deep sleep. Only an interrupt can wake it up.
void sleep(int ncycles)
{  
  nbr_remaining = ncycles; // defines how many cycles should sleep

  // Set sleep to full power down.  Only external interrupts or
  // the watchdog timer can wake the CPU!
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
 
  // Turn off the ADC while asleep.
  power_adc_disable();
 
  while (nbr_remaining > 0){ // while some cycles left, sleep!

  // Enable sleep and enter sleep mode.
  sleep_mode();

  // CPU is now asleep and program execution completely halts!
  // Once awake, execution will resume at this point if the
  // watchdog is configured for resume rather than restart
 
  // When awake, disable sleep mode
  sleep_disable();
 
  }
 
  // put everything on again
  power_all_enable();
 
}



// Humid accuracy +/- 5%
// Steady accuracy between 10-80
// example at 10/90 +/- 6%, 0/100 +/- 7.5%

// Temp accuracy +/- .5 degrees celsius
// Temp error increases more as we get farther from 25 cels.
// example: @ 0/50 degrees, +/- 1.2 degrees
