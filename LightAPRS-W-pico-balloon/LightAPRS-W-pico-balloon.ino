#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>
#include <GEOFENCE.h>       // Modified version of https://github.com/TomasTT7/TT7F-Float-Tracker/blob/master/Software/ARM_GEOFENCE.c
#include <LibAPRS.h>        //Modified version of https://github.com/markqvist/LibAPRS
#include <TinyGPS++.h>      //https://github.com/mikalhart/TinyGPSPlus
#include <LowPower.h>       //https://github.com/rocketscream/Low-Power
#include <Adafruit_BMP085.h>//https://github.com/adafruit/Adafruit-BMP085-Library
#include <si5351.h>         //https://github.com/etherkit/Si5351Arduino
#include <JTEncode.h>       //https://github.com/etherkit/JTEncode (JT65/JT9/JT4/FT8/WSPR/FSQ Encoder Library)
#include <TimeLib.h>        //https://github.com/PaulStoffregen/Time
#include <TimerThree.h>     //https://github.com/PaulStoffregen/TimerThree/

#define RfPDPin     19
#define GpsVccPin   18
#define RfPwrHLPin  21
#define RfPttPin    20
#define SiVccPin    0
#define BattPin     A2
#define PIN_DRA_RX  22
#define PIN_DRA_TX  23

#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false

#define GpsON         digitalWrite(GpsVccPin, LOW)//PNP
#define GpsOFF        digitalWrite(GpsVccPin, HIGH)
#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
#define RfPttON       digitalWrite(RfPttPin, HIGH)//NPN
#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define SiON          digitalWrite(SiVccPin, HIGH)//NPN
#define SiOFF         digitalWrite(SiVccPin, LOW)
#define AprsPinInput  pinMode(12,INPUT);pinMode(13,INPUT);pinMode(14,INPUT);pinMode(15,INPUT)
#define AprsPinOutput pinMode(12,OUTPUT);pinMode(13,OUTPUT);pinMode(14,OUTPUT);pinMode(15,OUTPUT)

#define DEVMODE       // Development mode. Uncomment to enable for debugging.
#define GROUNDTEST    // testing on the ground, ignore GPS (which is not working)
#define DO_APRS        // enables APRS transmissions
#define DO_WSPR        // enables WSPR transmissions

//******************************  APRS CONFIG **********************************
#ifdef DO_APRS
char    CallSign[7]="N6DM"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int8_t  CallNumber=11; //SSID http://www.aprs.org/aprs11/SSIDs.txt
char    Symbol='O'; // '/O' for balloon, '/>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool    alternateSymbolTable = false ; //false = '/' , true = '\'

char    comment[50] = "http://www.lightaprs.com"; // Max 50 char
char    StatusMessage[50] = "LightAPRS-W by TA2NHP & TA2MUN";
#endif
//*****************************************************************************

uint16_t  BeaconWait=50;  //seconds sleep for next beacon (HF or VHF). This is optimized value, do not change this if possible.
uint16_t  BattWait=60;    //seconds sleep if super capacitors/batteries are below BattMin (important if power source is solar panel) 
float     BattMin=4.5;    // min Volts to wake up.
#ifdef DO_APRS
float     DraHighVolt=6.0;// min Volts for radio module (DRA818V) to transmit (TX) 1 Watt, below this transmit 0.5 Watt.
#endif
float     GpsMinVolt=4.0; //min Volts for GPS to wake up. (important if power source is solar panel) 
#ifdef DO_WSPR
float     WsprBattMin=4.5; //min Volts for HF mradio module to transmit (TX) ~10 mW
#endif
//******************************  HF CONFIG *************************************

char hf_call[7] = "N6DM"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN

//#define WSPR_DEFAULT_FREQ       10140200UL //30m band
#define WSPR_DEFAULT_FREQ       14097200UL //20m band
//#define WSPR_DEFAULT_FREQ       18106100UL //17M band
//#define WSPR_DEFAULT_FREQ       21096100UL //15m band
//#define WSPR_DEFAULT_FREQ       24926100UL //12M band
//#define WSPR_DEFAULT_FREQ       28126100UL //10m band
//for all bands -> http://wsprnet.org/drupal/node/7352


// Supported modes, default HF mode is WSPR
enum mode {MODE_JT9, MODE_JT65, MODE_JT4, MODE_WSPR, MODE_FSQ_2, MODE_FSQ_3,
  MODE_FSQ_4_5, MODE_FSQ_6, MODE_FT8};

enum mode cur_mode = MODE_WSPR; //default HF mode

//supported other modes freq for 20m
#define JT9_DEFAULT_FREQ        14078700UL
#define FT8_DEFAULT_FREQ        14075000UL
#define JT65_DEFAULT_FREQ       14078300UL
#define JT4_DEFAULT_FREQ        14078500UL
#define FSQ_DEFAULT_FREQ        7105350UL     // Base freq is 1350 Hz higher than dial freq in USB

//*******************************************************************************


//******************************  APRS SETTINGS *********************************

//do not change WIDE path settings below if you don't know what you are doing :) 
uint8_t   Wide1=1; // 1 for WIDE1-1 path
uint8_t   Wide2=1; // 1 for WIDE2-1 path

/**
Airborne stations above a few thousand feet should ideally use NO path at all, or at the maximum just WIDE2-1 alone.  
Due to their extended transmit range due to elevation, multiple digipeater hops are not required by airborne stations.  
Multi-hop paths just add needless congestion on the shared APRS channel in areas hundreds of miles away from the aircraft's own location.  
NEVER use WIDE1-1 in an airborne path, since this can potentially trigger hundreds of home stations simultaneously over a radius of 150-200 miles. 
 */
uint8_t pathSize=2; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N
boolean autoPathSizeHighAlt = true; //force path to WIDE2-N only for high altitude (airborne) beaconing (over 1.000 meters (3.280 feet)) 

boolean beaconViaARISS = false; //there are no iGates in some regions (such as North Africa,  Oceans, etc) so try to beacon via ARISS (International Space Station) https://www.amsat.org/amateur-radio-on-the-iss/

boolean radioSetup = false;
boolean  aliveStatus = true; //for tx status message on first wake-up just once.

#ifdef DO_APRS
static char aprsTelemetryBuff[100];   // APRS telemetry buffer
uint16_t TxCount = 1; //increase +1 after every APRS transmission
#endif

//*******************************************************************************

//******************************  HF SETTINGS   *********************************

#define JT9_TONE_SPACING        174          // ~1.74 Hz
#define JT65_TONE_SPACING       269          // ~2.69 Hz
#define JT4_TONE_SPACING        437          // ~4.37 Hz
#define WSPR_TONE_SPACING       146          // ~1.46 Hz
#define FSQ_TONE_SPACING        879          // ~8.79 Hz
#define FT8_TONE_SPACING        625          // ~6.25 Hz

#define JT9_DELAY               576          // Delay value for JT9-1
#define JT65_DELAY              371          // Delay in ms for JT65A
#define JT4_DELAY               229          // Delay value for JT4A
#define WSPR_DELAY              683          // Delay value for WSPR
#define FSQ_2_DELAY             500          // Delay value for 2 baud FSQ
#define FSQ_3_DELAY             333          // Delay value for 3 baud FSQ
#define FSQ_4_5_DELAY           222          // Delay value for 4.5 baud FSQ
#define FSQ_6_DELAY             167          // Delay value for 6 baud FSQ
#define FT8_DELAY               159          // Delay value for FT8

#define CORRECTION              -3700         // Change this for your ref osc

// Global variables
unsigned long hf_freq;
char hf_message[13] = "NOCALL AA00";//for non WSPR modes, you don't have to change this, updated by hf_call and GPS location
char hf_loc[] = "AA00"; //for WSPR, updated by GPS location. You don't have to change this.
uint8_t dbm = 10;
uint8_t tx_buffer[255];
uint8_t symbol_count;
uint16_t tone_delay, tone_spacing;
volatile bool proceed = false;

boolean HFSent = false;
int32_t pressureAltitudeM;    // derived from barometric pressure reading

//*******************************************************************************


//******************************  GPS SETTINGS   *********************************

int16_t   GpsResetTime=1800; // originally 600 -- time to reset GPS, if GPS cannot fix our location

// GEOFENCE 
uint32_t GEOFENCE_APRS_frequency      = 144390000; //default frequency before geofencing. This variable will be updated based on GPS location.
uint32_t GEOFENCE_no_tx               = 0; 

boolean GpsFirstFix=false; //do not change this
boolean ublox_high_alt_mode_enabled = false; //do not change this
int16_t GpsInvalidTime=0; //do not change this
int32_t GpsValidTime=0;

//********************************************************************************

Si5351 si5351(0x60);
TinyGPSPlus gps;
Adafruit_BMP085 bmp;
JTEncode jtencode;

void setup() {
  wdt_enable(WDTO_8S);
  analogReference(INTERNAL2V56);
  pinMode(RfPDPin, OUTPUT);
  pinMode(GpsVccPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PIN_DRA_TX, INPUT);
  pinMode(SiVccPin, OUTPUT);

  RfOFF;
  GpsOFF;
  RfPwrLow;
  RfPttOFF;
  SiOFF;
  AprsPinInput;

  Serial.begin(57600); //Arduino serial
  Serial1.begin(9600); //GPS serial
#if defined(DEVMODE)
  Serial.println(F("\n\n\n\n_______________ Start Setup ____________"));
#endif

#ifdef DO_APRS
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign,CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setMessageDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_useAlternateSymbolTable(alternateSymbolTable); 
  APRS_setSymbol(Symbol);
  //increase following value (for example to 500UL) if you experience packet loss/decode issues.   
  APRS_setPreamble(350UL);
  APRS_setPathSize(pathSize);
  AprsPinInput;
#endif

  bmp.begin();

}

void loop() {
  wdt_reset();

  // read battery, temperature and barometer values
  float batteryV = readBatt();

#ifdef DEVMODE
  printSensors(batteryV);
#endif

  if (batteryV > BattMin) {
    if (aliveStatus) {
#if defined(DEVMODE)
      Serial.println(F("--- Sending status via APRS"));
#endif
      //sendStatusViaAPRS();
#if defined(DEVMODE)
      Serial.println(F("--- Status sent via APRS"));
#endif
      aliveStatus = false;

      while (readBatt() < BattMin) {
        sleepSeconds(BattWait); 
      }
    }

#ifdef GROUNDTEST
    enterGroundTestGPSData();
#else
    updateGpsData(1000);
#endif
    gpsDebug();

    if(gps.location.isValid() && gps.location.age()<1000){
      GpsInvalidTime=0;
      GpsValidTime++;
    }else{
      GpsValidTime=0;
      GpsInvalidTime++;
      if(GpsInvalidTime > GpsResetTime){
        GpsOFF;
        ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.
        wdt_reset();
        delay(1000);
        GpsON;
        GpsInvalidTime = 0;
        GpsValidTime = 0;

        gps.location = TinyGPSLocation();
        gps.date = TinyGPSDate();
        gps.time = TinyGPSTime();
        gps.speed = TinyGPSSpeed();
        gps.course = TinyGPSCourse();
        gps.altitude = TinyGPSAltitude();
        gps.satellites = TinyGPSInteger();
        gps.hdop = TinyGPSDecimal();
      }
    }

    if ((gps.location.age() < 1000 || gps.location.isUpdated()) && gps.location.isValid()) {
      if (gps.satellites.isValid() && gps.satellites.value() >= 3) {
        GpsOFF;
        GpsFirstFix = true;
        ublox_high_alt_mode_enabled = false; //gps sleep mode resets high altitude mode.

#ifndef DO_WSPR
        if (0) {
#endif
#ifdef DO_WSPR
#if defined(DEVMODE)
          Serial.println(F("--- Before WSPR Check"));
#endif        
        // preparations for HF starts one minute before TX time at minute 3, 7, 13, 17, 23, 27, 33, 37, 43, 47, 53 or 57. No APRS TX during this period...
        if (readBatt() > WsprBattMin && timeStatus() == timeSet && ((minute() % 10 == 3) || (minute() % 10 == 7)) ) { 
          GridLocator(hf_loc, gps.location.lat(), gps.location.lng());
          sprintf(hf_message,"%s %s",hf_call,hf_loc);

#if defined(DEVMODE)
          Serial.println(F("--- Digital HF Mode Prepearing"));
          Serial.println(hf_loc);
#endif
          HFSent = false;
          //HF transmission starts at minute 4, 8, 14, 18, 24, 28, 34, 38, 44, 48, 54 or 58 
          while (((minute() % 10 != 4) || (minute() % 10 != 8)) && second() != 0) {
            wdt_reset();
          }
#if defined(DEVMODE)
          Serial.println(F("--- Digital HF Mode Sending"));
#endif          
          encode();

          HFSent = true;
#if defined(DEVMODE)
          Serial.println(F("--- Digital HF Mode Sent"));
#endif       
#endif // DO_WSPR

        } else {
#ifdef DO_APRS
          updatePosition();
          updateTelemetry();
          //APRS frequency isn't the same for the whole world. (for pico balloon only)
          if (!radioSetup || TxCount == 200) {
            configureFreqbyLocation();
          }

          if(autoPathSizeHighAlt && gps.altitude.feet()>3000){
            //force to use high altitude settings (WIDE2-n)
            APRS_setPathSize(1);
          } else {
            //use default settings
            APRS_setPathSize(pathSize);
          }

          //in some countries Airborne APRS is not allowed. (for pico balloon only)
          if (isAirborneAPRSAllowed()) {
            sendLocationViaAPRS();
          }
#endif // DO_APRS

          freeMem();
          Serial.flush();
          //If two minutes time slot is close, sleep less than default. 
          if (timeStatus() != timeNotSet && ((minute() % 10 == 2) || (minute() % 10 == 6))){
             sleepSeconds(60 - second());
          } else {
             sleepSeconds(BeaconWait);
          }

        }

      } else {
#if defined(DEVMODE)
        Serial.println(F("*** Not enough satellites"));
#endif
      }
    } 
  } else {
    sleepSeconds(BattWait);
  }

}

void aprs_msg_callback(struct AX25Msg *msg) {
  //do not remove this function, necessary for LibAPRS.
}

#ifdef DEVMODE
void printSensors(float batteryV) {
  float temperatureC = bmp.readTemperature();
  int32_t baroPressureHPa = lroundf(bmp.readPressure() / 100.0); //Pa to hPa
  pressureAltitudeM = toPressureAltitude(baroPressureHPa, temperatureC);

  char line[100];
  char s1[20], s2[20];
  dtostrf(batteryV, 4, 1, s1);
  dtostrf(temperatureC, 4, 1, s2);
  sprintf(line, "--- battery: %sv, temperature: %sºC, pressure: %ld, altitude: %ldm",
          s1, s2, baroPressureHPa, pressureAltitudeM);
  Serial.println(line);

  if (GpsInvalidTime > 0) {
    sprintf(line, "*** GPS INVALID: %d seconds", GpsInvalidTime);
  } else {
    sprintf(line, "--- GPS Valid: %d seconds", GpsValidTime);
  }
  Serial.println(line);
}
#endif  // DEVMODE

void sleepSeconds(int sec) {
  if (GpsFirstFix){//sleep gps after first fix
      GpsOFF;
      ublox_high_alt_mode_enabled = false;
  }
  RfOFF;
  RfPttOFF;
  SiOFF;
#if defined(DEVMODE)
  Serial.print(F("--- Sleeping for "));
  printInt(sec,true,4);
  Serial.println(F(" seconds.\n"));
  Serial.flush();
#endif

  wdt_disable();
  int secsLeft = (sec * 88) / 100;    // reduce by 12% due to overhead of powering down
  while (secsLeft > 0) {
    if (readBatt() < GpsMinVolt){
      GpsOFF;
      ublox_high_alt_mode_enabled = false;
    } 

    if (secsLeft >= 8) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_ON);
      secsLeft -= 8;
    } else {
      LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);
      secsLeft -= 1;
    }
  }
  wdt_enable(WDTO_8S);
  wdt_reset();

  adjustTime(sec);  // fix time, because millis() doesn't advance during sleep
}

// computes approximate altitude based on barometric pressure and temperature
// source: https://keisan.casio.com/exec/system/1224585971
// added: 9/4/2021 by N6DM
//
// Note: If the altitude is more than 11km high above sea level, the
//       hypsometric formula cannot be applied because the temperature
//       lapse rate varies considerably with altitude)
//
int32_t toPressureAltitude(int32_t pressureHPa, float tempC) {
  const float stdSeaLevelPressHPa = 1013.25;  // hPa = hecto Pascal (aka millibars)
  float tmp1 = stdSeaLevelPressHPa / float(pressureHPa);
  float tmp2 = pow(tmp1, 1.0 / 5.257) - 1.0;
  float tmp3 = tmp2 * (tempC + 273.15);
  float altitudeMeters = tmp3 / 0.0065;
  return int32_t(lroundf(altitudeMeters));
}

#ifdef DO_APRS
boolean isAirborneAPRSAllowed() {

  float tempLat = gps.location.lat();
  float tempLong = gps.location.lng();

  GEOFENCE_position(tempLat, tempLong);

  boolean airborne = true;

  if (GEOFENCE_no_tx == 1) {
    airborne = false;
  }

  return airborne;
}

boolean inARISSGeoFence(float tempLat, float tempLong) {
  boolean ariss = false;
  //North Africa
  if(tempLat>0 && tempLat<32 && tempLong>0 && tempLong<32){ariss = true;}
  //North Pacific
  if(tempLat>28 && tempLat<50 && tempLong>-180 && tempLong<-130){ariss = true;}
  //North Atlantic
  if(tempLat>25 && tempLat<42 && tempLong>-60 && tempLong<-33){ariss = true;} 

  return ariss;
}

void configureFreqbyLocation() {

  float tempLat = gps.location.lat();
  float tempLong = gps.location.lng();
  boolean success = false;
  
  if(beaconViaARISS && inARISSGeoFence(tempLat, tempLong)) {
    APRS_setPath1("ARISS", Wide1);
    APRS_setPath2("WIDE2", Wide2);
    APRS_setPathSize(2);
    success = configDra818("145.8250"); 
     
  } else {
    GEOFENCE_position(tempLat,tempLong);  
    float dividedFreq = GEOFENCE_APRS_frequency / 1000000.f;
    char aprsFreq_buff[9];
    dtostrf(dividedFreq, 8, 4, aprsFreq_buff);
    success = configDra818(aprsFreq_buff);    
  }

  radioSetup = success;
}

boolean configDra818(char *freq)
{
  char cmd[50];
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  //strcpy(cmd, "AT+DMOCONNECT");
  
  SoftwareSerial Serial_dra(PIN_DRA_RX, PIN_DRA_TX);
  Serial_dra.begin(9600);
  delay(100);
  RfON;
  delay(3000);

  boolean success = false;
  int retriesLeft = 3;
  while (retriesLeft > 0) {
#ifdef DEVMODE
    Serial.print(F("--- Config DRA: "));
    Serial.println(cmd);
#endif
    // send command to configure frequency
    Serial_dra.println(cmd);    

    // restart watchdog timer so we have enough time to finish
    wdt_reset();    
  
    success = waitForAckFromDRA(3000, &Serial_dra);
    if (success) break;
    retriesLeft--;
  }
 
  Serial_dra.end();
  RfOFF;
  pinMode(PIN_DRA_TX, INPUT);
  
#if defined(DEVMODE)
  if (success) {
    Serial.println(F("--- Frequency updated...")); 
  } else {
    Serial.println(F("*** ERR: Frequency update error!"));
  }
#endif

  return success; 
}

boolean waitForAckFromDRA(uint16_t timeLimitMS, SoftwareSerial* Serial_dra) 
{
  const uint32_t startMS = millis();
  uint32_t elapsedMS = 0;     // milliseconds spent waiting for ACK
  boolean success = false;
  
  char reply[80];     // reply bytes, not null terminated
  int replyLen = 0;   // length of reply string content
  
  while (elapsedMS < timeLimitMS)
  {

    // get next character in reply text
    //
    if (Serial_dra->available() == 0) {
      // still waiting, update elapsed time
      elapsedMS = millis() - startMS;

      // still waiting for reply, timeout if too long
      if (elapsedMS >= timeLimitMS) {
#ifdef DEVMODE
        printReply(reply, replyLen);
        Serial.println(F("*** ERR: timed out waiting for DRA reply"));
#endif
        return false;     //********* TIMEOUT, failed to get ACK
      }
     
    } else {
      // read the next character from DRA into reply buffer
      reply[replyLen++] = Serial_dra->read();
 
      // look for LF at end of reply message
      if (reply[replyLen] == 0x0A) {
        // check "0" or "1" in reply to see if really successful
        if (reply[replyLen-2] == '1') {
#ifdef DEVMODE
          printReply(reply, replyLen);
          Serial.println(F("--- Success DRA configured"));
#endif
          return true;    //--------- SUCCESS, GOT ACK
        } else if (reply[replyLen-2] == '0') {
#ifdef DEVMODE
          printReply(reply, replyLen);
          Serial.println(F("*** ERR: DRA return 0"));
#endif
          return false;   //--------- FAILED, GOT NACK
        }
      }

      if (replyLen >= sizeof(reply)) {
        // reply buffer is full, discard older data, keep recent data
        int discardLen = sizeof(reply) / 2;
#ifdef DEVMODE
        printReply(reply, discardLen);
#endif
        char* start = &reply[0];
        char* middle = &reply[discardLen];
        memmove(start, middle, replyLen-discardLen);
        replyLen -= discardLen;
      }
    }
  }  // while loop
}

void updatePosition() {
  // Convert and set latitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[S,N].
  char latStr[10];
  int temp = 0;

  double d_lat = gps.location.lat();
  double dm_lat = 0.0;

  if (d_lat < 0.0) {
    temp = -(int)d_lat;
    dm_lat = temp * 100.0 - (d_lat + temp) * 60.0;
  } else {
    temp = (int)d_lat;
    dm_lat = temp * 100 + (d_lat - temp) * 60.0;
  }

  dtostrf(dm_lat, 7, 2, latStr);

  if (dm_lat < 1000) {
    latStr[0] = '0';
  }

  if (d_lat >= 0.0) {
    latStr[7] = 'N';
  } else {
    latStr[7] = 'S';
  }

  APRS_setLat(latStr);

  // Convert and set longitude NMEA string Degree Minute Hundreths of minutes ddmm.hh[E,W].
  char lonStr[10];
  double d_lon = gps.location.lng();
  double dm_lon = 0.0;

  if (d_lon < 0.0) {
    temp = -(int)d_lon;
    dm_lon = temp * 100.0 - (d_lon + temp) * 60.0;
  } else {
    temp = (int)d_lon;
    dm_lon = temp * 100 + (d_lon - temp) * 60.0;
  }

  dtostrf(dm_lon, 8, 2, lonStr);

  if (dm_lon < 10000) {
    lonStr[0] = '0';
  }
  if (dm_lon < 1000) {
    lonStr[1] = '0';
  }

  if (d_lon >= 0.0) {
    lonStr[8] = 'E';
  } else {
    lonStr[8] = 'W';
  }

  APRS_setLon(lonStr);
}

void updateTelemetry() {
  sprintf(aprsTelemetryBuff, "%03d", gps.course.isValid() ? (int)gps.course.deg() : 0);
  aprsTelemetryBuff[3] += '/';
  sprintf(aprsTelemetryBuff + 4, "%03d", gps.speed.isValid() ? (int)gps.speed.knots() : 0);
  aprsTelemetryBuff[7] = '/';
  aprsTelemetryBuff[8] = 'A';
  aprsTelemetryBuff[9] = '=';
  //sprintf(aprsTelemetryBuff + 10, "%06lu", (long)gps.altitude.feet());

  //fixing negative altitude values causing display bug on aprs.fi
  float tempAltitude = gps.altitude.feet();

  if (tempAltitude > 0) {
    //for positive values
    sprintf(aprsTelemetryBuff + 10, "%06lu", (long)tempAltitude);
  } else {
    //for negative values
    sprintf(aprsTelemetryBuff + 10, "%06d", (long)tempAltitude);
  }

  aprsTelemetryBuff[16] = ' ';
  sprintf(aprsTelemetryBuff + 17, "%03d", TxCount);
  aprsTelemetryBuff[20] = 'T';
  aprsTelemetryBuff[21] = 'x';
  aprsTelemetryBuff[22] = 'C';
  aprsTelemetryBuff[23] = ' '; float tempC = bmp.readTemperature();//-21.4;//
  dtostrf(tempC, 6, 2, aprsTelemetryBuff + 24);
  aprsTelemetryBuff[30] = 'C';
  aprsTelemetryBuff[31] = ' '; float pressure = bmp.readPressure() / 100.0; //Pa to hPa
  dtostrf(pressure, 7, 2, aprsTelemetryBuff + 32);
  aprsTelemetryBuff[39] = 'h';
  aprsTelemetryBuff[40] = 'P';
  aprsTelemetryBuff[41] = 'a';
  aprsTelemetryBuff[42] = ' ';
  dtostrf(readBatt(), 5, 2, aprsTelemetryBuff + 43);
  aprsTelemetryBuff[48] = 'V';
  aprsTelemetryBuff[49] = ' ';
  sprintf(aprsTelemetryBuff + 50, "%02d", gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  aprsTelemetryBuff[52] = 'S';
  aprsTelemetryBuff[53] = ' ';

  sprintf(aprsTelemetryBuff + 54, "%s", comment);

#if defined(DEVMODE)
  Serial.print(F("--- APRS Telemetry: "));
  Serial.println(aprsTelemetryBuff);
#endif

}

void sendLocationViaAPRS() {

#if defined(DEVMODE)
  Serial.println(F("--- APRS Location sending with comment"));
#endif
  if ((readBatt() > DraHighVolt) && (readBatt() < 10)) RfPwrHigh; //DRA Power 1 Watt
  else RfPwrLow; //DRA Power 0.5 Watt

  const bool timeValid = gps.time.isValid();
  int hh = timeValid ? gps.time.hour() : 0;
  int mm = timeValid ? gps.time.minute() : 0;
  int ss = timeValid ? gps.time.second() : 0;

  char timestamp_buff[8];
  sprintf(timestamp_buff, "%02d%02d%02dh", hh, mm, ss);

  wdt_reset();    // restart watchdog timer so we have enough time to finish

  AprsPinOutput;
  RfON;
  delay(2000);
  RfPttON;
  delay(1000);

  APRS_sendLocWtTmStmp(aprsTelemetryBuff, strlen(aprsTelemetryBuff), timestamp_buff);
  
  delay(50);
  while (digitalRead(1)) {;} //LibAprs TX Led pin PB1
  delay(50);
  RfPttOFF;
  RfOFF;
  AprsPinInput;
#if defined(DEVMODE)
  Serial.println(F("--- APRS Location sent with comment"));
#endif

  TxCount++;
}

void sendStatusViaAPRS() {
  if ((readBatt() > DraHighVolt) && (readBatt() < 10)) RfPwrHigh; //DRA Power 1 Watt
  else RfPwrLow; //DRA Power 0.5 Watt
  AprsPinOutput;
  RfON;
  delay(2000);
  RfPttON;
  delay(1000);

  APRS_sendStatus(StatusMessage, strlen(StatusMessage));
  
  delay(50);
  while (digitalRead(1)) {;} //LibAprs TX Led pin PB1
  delay(50);
  RfPttOFF;
  RfOFF;
  AprsPinInput;
#if defined(DEVMODE)
  Serial.println(F("--- Status sent"));
#endif

  TxCount++;
}
#endif    // DO_APRS

static void updateGpsData(int ms)
{
#ifdef DEVMODE
   Serial.println(F("--- GPS Update START\n"));
#endif
  GpsON;

  if(!ublox_high_alt_mode_enabled){
      //enable ublox high altitude mode
      delay(100);   // pause 100 ms
      setGPS_DynamicModel6();
      #if defined(DEVMODE)
        Serial.println(F("--- ublox DynamicModel6 enabled..."));
      #endif      
      ublox_high_alt_mode_enabled = true;
  }

  while (!Serial1) {
    delayMicroseconds(1); // wait for serial port to connect.
  }

  unsigned long start = millis();
  unsigned long bekle = 0;
  do
  {
    while (Serial1.available() > 0) {
      char c;
      c = Serial1.read();
      bool goodSentence = gps.encode(c);
#ifdef DEVMODE
      if (!goodSentence) {
        Serial.print(c);
      }
#endif
      bekle = millis();
    }
    if (gps.time.isValid())
    {
      setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), NULL, NULL, NULL);

    }
    if (bekle != 0 && bekle + 10 < millis()) break;
  } while (millis() - start < ms);
#ifdef DEVMODE
   Serial.println(F("--- GPS Update DONE"));
#endif
}

#ifdef GROUNDTEST
void enterGroundTestGPSData() {
  Serial.println(F("--- Start GROUND TEST"));
  delay(950);

  int year = 20;
  int month = 12;
  int day = 31;
  int hh = 18;
  int mm = 0;
  int ss = 0;
  
  char UTCTimeStr[10];
  char UTCDateStr[10];
  
  if (timeStatus() == timeSet) {
      Serial.print(F("--- RTC already set: "));
      hh = hour();
      mm = minute();
      ss = second();
  } else {
    Serial.print(F("--- RTC not yet set: "));
  }
  sprintf(UTCTimeStr, "%02d%02d%02d", hh, mm, ss);
  Serial.println(UTCTimeStr);

  // date string
  sprintf(UTCDateStr, "%02d%02d%02d", day, month, year);

  // ground test in progress, ignore GPS, used canned GPS sentences
  char sentence[120];
  sprintf(sentence, "$GPRMC,%s,A,3300.00,N,11700.00,W,000.0,000.0,%s,014.0,E", UTCTimeStr, UTCDateStr);
  encodeNMEA(sentence);
  
  sprintf(sentence, "$GPGLL,3300.00,N,11700.00,W,%s,A", UTCTimeStr);
  encodeNMEA(sentence);

  sprintf(sentence, "$GPGGA,%s,3300.00,N,11700.00,W,1,03,2,250,M,0,M,,", UTCTimeStr);
  encodeNMEA(sentence);
  
  if (gps.time.isValid()) {
      Serial.println(F("--- Valid GPS time"));
      if (timeStatus() == timeNotSet) {
        Serial.println(F("--- RTC wasn't set, setting now using GPS time"));
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), day, month, year);
      }
  } else {
      Serial.println(F("*** INVALID GPS TIME"));
  }
}

void encodeNMEA(const char* const str) {
  uint8_t parity = 0;

  // send NMEA string to TinyGPSPlus
  for (int i=0; i<strlen(str); i++) { 
    char c = str[i];
    if (c != '$') {
      parity ^= c;   // compute parity
    }
    gps.encode(c);
  }

  // convert parity to two hex characters
  char p1 = toHex((parity & 0xF0) >> 4);
  char p2 = toHex(parity & 0x0F);

  // send NMEA parity string to finish
  gps.encode('*');
  gps.encode(p1);
  gps.encode(p2);
  bool valid = gps.encode('\n');
  
  if (!valid) {
      Serial.print(F("*** ERR: invalid sentence: "));
  } else {
      Serial.print(F("--- Valid: "));
  }
  char debugMsg[120];
  sprintf(debugMsg, "%s$%c%c", str, p1, p2);
  Serial.println(debugMsg);
}

char toHex(uint8_t val) {
  if (val < 10) {
    return '0' + val;
  } else {
    return 'A' + val - 10;
  }
}
#endif

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0;
  do {
    value = analogRead(BattPin);
    delay(5);// ilk açılışta hatalı ölçüm yapmaması için
    value = analogRead(BattPin);
    //value = value - 8; // direnç tolerans düzeltmesi
    value = (value * 2.56) / 1024.0;
    value = value / (R2 / (R1 + R2));
  } while (value > 16.0);
  return value ;
}

void Timer3Tick(void)
{
  proceed = true;
}

void encode()
{
  wdt_reset();
  SiON;
  delay(20);
  si5351.init(SI5351_CRYSTAL_LOAD_0PF, 0, CORRECTION);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA); // Set for max power if desired
  si5351.output_enable(SI5351_CLK0, 0);
  uint8_t i;

  switch(cur_mode)
  {
    case MODE_JT9:
      hf_freq = JT9_DEFAULT_FREQ;
      symbol_count = JT9_SYMBOL_COUNT; // From the library defines
      tone_spacing = JT9_TONE_SPACING;
      tone_delay = JT9_DELAY;
      break;
    case MODE_JT65:
      hf_freq = JT65_DEFAULT_FREQ;
      symbol_count = JT65_SYMBOL_COUNT; // From the library defines
      tone_spacing = JT65_TONE_SPACING;
      tone_delay = JT65_DELAY;
      break;
    case MODE_JT4:
      hf_freq = JT4_DEFAULT_FREQ;
      symbol_count = JT4_SYMBOL_COUNT; // From the library defines
      tone_spacing = JT4_TONE_SPACING;
      tone_delay = JT4_DELAY;
      break;
    case MODE_WSPR:
      hf_freq = WSPR_DEFAULT_FREQ;
      symbol_count = WSPR_SYMBOL_COUNT; // From the library defines
      tone_spacing = WSPR_TONE_SPACING;
      tone_delay = WSPR_DELAY;
      break;
    case MODE_FT8:
      hf_freq = FT8_DEFAULT_FREQ;
      symbol_count = FT8_SYMBOL_COUNT; // From the library defines
      tone_spacing = FT8_TONE_SPACING;
      tone_delay = FT8_DELAY;
      break;
    case MODE_FSQ_2:
      hf_freq = FSQ_DEFAULT_FREQ;
      tone_spacing = FSQ_TONE_SPACING;
      tone_delay = FSQ_2_DELAY;
      break;
    case MODE_FSQ_3:
      hf_freq = FSQ_DEFAULT_FREQ;
      tone_spacing = FSQ_TONE_SPACING;
      tone_delay = FSQ_3_DELAY;
      break;
    case MODE_FSQ_4_5:
      hf_freq = FSQ_DEFAULT_FREQ;
      tone_spacing = FSQ_TONE_SPACING;
      tone_delay = FSQ_4_5_DELAY;
      break;
    case MODE_FSQ_6:
      hf_freq = FSQ_DEFAULT_FREQ;
      tone_spacing = FSQ_TONE_SPACING;
      tone_delay = FSQ_6_DELAY;
      break;
  }
  set_tx_buffer();

  // Now transmit the channel symbols
  if (cur_mode == MODE_FSQ_2 || cur_mode == MODE_FSQ_3 || cur_mode == MODE_FSQ_4_5 || cur_mode == MODE_FSQ_6)
  {
    uint8_t j = 0;
    while (tx_buffer[j++] != 0xff);
    symbol_count = j - 1;
  }

  // Reset the tone to the base frequency and turn on the output
  si5351.output_enable(SI5351_CLK0, 1);

  uint32_t timer_period = tone_delay;
  timer_period *= 1000;
  Timer3.initialize(timer_period);
  Timer3.attachInterrupt(Timer3Tick);
  Timer3.restart();

  for (i = 0; i < symbol_count; i++)
  {
    si5351.set_freq((hf_freq * 100) + (tx_buffer[i] * tone_spacing), SI5351_CLK0);
    proceed = false;
    while (!proceed);
    wdt_reset();
  }
  Timer3.stop();
  // Turn off the output
  si5351.output_enable(SI5351_CLK0, 0);
  SiOFF;
}

void set_tx_buffer()
{
  // Clear out the transmit buffer
  memset(tx_buffer, 0, 255);

  // Set the proper frequency and timer CTC depending on mode
  switch(cur_mode)
  {
    case MODE_JT9:
      jtencode.jt9_encode(hf_message, tx_buffer);
      break;
    case MODE_JT65:
      jtencode.jt65_encode(hf_message, tx_buffer);
      break;
    case MODE_JT4:
      jtencode.jt4_encode(hf_message, tx_buffer);
      break;
    case MODE_WSPR:
      jtencode.wspr_encode(hf_call, hf_loc, dbm, tx_buffer);
      break;
    case MODE_FT8:
      jtencode.ft8_encode(hf_message, tx_buffer);
      break;
    case MODE_FSQ_2:
    case MODE_FSQ_3:
    case MODE_FSQ_4_5:
    case MODE_FSQ_6:
      jtencode.fsq_dir_encode(hf_call, "n0call", ' ', "hello world", tx_buffer);
      break;
  }
}

void GridLocator(char *dst, float latt, float lon) {
  int o1, o2;
  int a1, a2;
  float remainder;
  // longitude
  remainder = lon + 180.0;
  o1 = (int)(remainder / 20.0);
  remainder = remainder - (float)o1 * 20.0;
  o2 = (int)(remainder / 2.0);
  // latitude
  remainder = latt + 90.0;
  a1 = (int)(remainder / 10.0);
  remainder = remainder - (float)a1 * 10.0;
  a2 = (int)(remainder);

  dst[0] = (char)o1 + 'A';
  dst[1] = (char)a1 + 'A';
  dst[2] = (char)o2 + '0';
  dst[3] = (char)a2 + '0';
  dst[4] = (char)0;
}

void freeMem() {
#if defined(DEVMODE)
  Serial.print(F("--- Free RAM: ")); Serial.print(freeMemory()); Serial.println(F(" byte"));
#endif
}

void gpsDebug() {
#if defined(DEVMODE)
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  RX    RX       Fail  OK"));
  Serial.println(F("-----------------------------------------------------------------------------------------------------------------"));

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);

  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);

  printDateTime(gps.date, gps.time);  // prints date, time and age

  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 4);
  printInt(gps.passedChecksum(), true, 5);
  Serial.println();

#endif
}

static void printFloat(float val, bool valid, int len, int prec)
{
#if defined(DEVMODE)
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); 
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
#endif
}

static void printInt(unsigned long val, bool valid, int len)
{
#if defined(DEVMODE)
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
#endif
}

#ifdef DEVMODE
static void printReply(char* reply, int replyLen) 
{
  if (replyLen>0) {
    Serial.print(F("DRA REPLY: "));
    printStringWHex(reply, replyLen);
    Serial.println();
  }
}

static void printStringWHex(char* str, int length)
{
  for (int i=0; i<length; i++) {
    char ch = str[i];
    if (ch <= ' ') {
      printHex(ch);
    } else {
      Serial.print(ch);  
    }
  }
}

static void printHex(uint16_t val) 
{
  uint16_t upper4 = (val & 0xF0) >> 4;
  uint16_t lower4 = (val & 0x0F);
  char hex[3];
  hex[0] = (upper4 <= 9) ? '0'+upper4 : 'A'+upper4-10;
  hex[1] = (lower4 <= 9) ? '0'+lower4 : 'A'+lower4-10;
  hex[2] = 0;
  Serial.print(hex);
}
#endif

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
#if defined(DEVMODE)
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
#endif
}

static void printStr(const char *str, int len)
{
#if defined(DEVMODE)
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
#endif
}

//following GPS code from : https://github.com/HABduino/HABduino/blob/master/Software/habduino_v4/habduino_v4.ino
void setGPS_DynamicModel6()
{
  int gps_set_sucess = 0;
  uint8_t setdm6[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
    0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };

  while (!gps_set_sucess)
  {
    sendUBX(setdm6, sizeof(setdm6) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setdm6);
  }
}

void sendUBX(uint8_t *MSG, uint8_t len) {
  Serial1.flush();
  Serial1.write(0xFF);
  delay(500);
  for (int i = 0; i < len; i++) {
    Serial1.write(MSG[i]);
  }
}

boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();

  // Construct the expected ACK packet
  ackPacket[0] = 0xB5; // header
  ackPacket[1] = 0x62; // header
  ackPacket[2] = 0x05; // class
  ackPacket[3] = 0x01; // id
  ackPacket[4] = 0x02; // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2]; // ACK class
  ackPacket[7] = MSG[3]; // ACK id
  ackPacket[8] = 0; // CK_A
  ackPacket[9] = 0; // CK_B

  // Calculate the checksums
  for (uint8_t ubxi = 2; ubxi < 8; ubxi++) {
    ackPacket[8] = ackPacket[8] + ackPacket[ubxi];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {
    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      return false;
    }

    // Make sure data is available to read
    if (Serial1.available()) {
      b = Serial1.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
      }
      else {
        ackByteID = 0; // Reset and look again, invalid order
      }
    }
  }
}
