/*
 Copyright (c) 2015, Cullen Jennings <fluffy@iii.ca> All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of Cullen Jennings nor the names of its contributors may
 be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,::w BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// TODO - check what happens sensor error all way back to spray wind

// TODO - shutdown at min battery voltage

// TODO - compute avg and max wind over last 10 min measurements in an array

// TODO - time since last sat TX likely wrong

// TODO - number sat tx retries could use adjustment


#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include <Wire.h>


#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


static const int btnPin = 2;
static const int eBtnPin = 3;
static const int auxPwrEnablePin = 4;

static const int satPwrEnablePin = 7;
static const int satTxPin = 8;
static const int satRxPin = 9;

static const int ledPin = 13;

static const int windPin = 0;
static const int batSensePin = 1;


static const byte displayAddress = 0x71; // for LED display
static const int rtcAddress = 0x68; // See data sheet for DS1307


SoftwareSerial satSerial( satRxPin, satTxPin );
IridiumSBD sat( satSerial, satPwrEnablePin );

#if 0
#define DEBUG( X ) Serial.println( X );
#define DEBUG_NOCR( X ) Serial.print( X );
#define DEBUG2( X, Y ) Serial.print( X ); Serial.println( Y );
#define DEBUG_NOCR2( X,Y ) Serial.print( X) ; Serial.print( Y );
#else
#define DEBUG( X ) ;
#define DEBUG_NOCR( X ) ;
#define DEBUG2( X, Y ) ;
#define DEBUG_NOCR2( X,Y ) ;
#endif


byte disableDisplay = 0;
byte disableRTC = 0;
byte disableSat = 0;

byte satActive = 0;
byte rtcActive = 0;
byte windActive = 0;
byte batActive = 0;
volatile byte dispActive = 0;

/****   Display stuff  ****/
volatile unsigned long dispStartTime = 0;
volatile byte dispItem = 0;


byte prevHour = -1;
byte prevMinute = -1;
byte prevSec = -1;


int numDeepSleeps = 0; // number of times powered down since last RTC sync

void setup()
{
  Serial.begin(9600);
  //Serial.begin(19200);

  DEBUG("Setup");

  pwrSetup();
  btnSetup();

  auxPowerOn();
  delay( 200 );

  Wire.begin();
  analogReference( DEFAULT ); // 3.3 V

  if (0) // only use this for intial set time of RTC clock
  {
    rtcSetTime();
  }

  batSetup();

  if ( batGetVoltageX10() < 90 )
  {
    DEBUG("bat voltage low - dispable SAT, RTC and DISPLAY");
    disableDisplay = 1;
    disableRTC = 1;
    disableSat = 1;
  }

  // aux power must be on for theses setups
  satSetup();
  rtcSetup();
  windSetup();
  dispSetup();

  rtcStart();
  windStart();
  batStart();

  if (1)
  {
    while ( rtcActive ) // wait to get a valid time before really starting
    {
      rtcRun();
    }

    rtcGetTime( &prevHour, &prevMinute, &prevSec);
  }

  dispStart(); // don't start till after have a time or rtc remains active until display is not

  setupSleep();

  DEBUG( "Done SETUP -----------------" );
}


void loop()
{
  //DEBUG_NOCR(".");
  delay(10);
  runSched();
}


/* power managment stuff */

byte auxPowerStatus = 0;

void pwrSetup()
{
  DEBUG("in pwrSetup");

  pinMode( auxPwrEnablePin, OUTPUT );
  auxPowerStatus = 0;
  auxPowerOn();
}

void auxPowerOn()
{
  if ( auxPowerStatus )
  {
    return;
  }
  //DEBUG("turn ON aux power ");
  digitalWrite( auxPwrEnablePin, HIGH );
  auxPowerStatus = 1;
}

void auxPowerOff()
{
  // TODO - sleep sat and check sleeping before power down ?
  if ( !auxPowerStatus )
  {
    return;
  }
  //DEBUG("turn OFF aux power ");
  digitalWrite( auxPwrEnablePin, LOW );
  auxPowerStatus = 0;
}


/* scheduler stuff */
unsigned long nowTime;
volatile byte stopSleep = 0;

void btnSetup()
{
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(0, btnPress, FALLING); // white button wire
}

volatile unsigned long lastBtnPressTime = 0;

void btnPress()
{
  // debounce
  long now = millis();
  if ( now < lastBtnPressTime + 250 )
  {
    return;
  }
  lastBtnPressTime = now;

  stopSleep = 1;
  if ( !dispActive )
  {
    dispStart();
  }
  else
  {
    dispNext();
  }
}



void runSched() {
  //DEBUG( "In runSched" );
  //DEBUG_NOCR( "." );

  nowTime = millis();
  //unsigned long t = (nowTime / 60000) % 60;
  byte nowMinute, nowSec, nowHour;
  rtcGetTime( &nowHour, &nowMinute, &nowSec);

  //DEBUG2( "nowMinute  = ", nowMinute );
  //DEBUG2( "prevMinute = ", prevMinute );

  if ( nowMinute != prevMinute )
  {
    DEBUG( "Scheudle start wind" );
    windStart();
  }

  if ( ((nowMinute + 1) / 15) != ((prevMinute + 1) / 15) ) // ever 15 min, but 1 min before satStart
  {
    DEBUG( "Scheudle start bat" );
    batStart();
  }

  if ( ((nowMinute + 1) / 5) != ((prevMinute + 1) / 5) ) // ever 5 min, but 1 min before satStart
  {
    DEBUG( "Scheudle start rtc" );
    rtcStart();
  }

  if ( 0 ) // debug special sat tx 15 seconds after power up
  {
    static byte firstTime = 1;
    if ( firstTime  && ( nowTime > 15 * 1000 ) )
    {
      satStart();
      firstTime = 0;
    }
  }

  if (1) // normal satTX schedule
  {
    if ( (nowHour >= 7) && (nowHour <= 16 ) )
    {
      // day
      if ( ( (nowMinute / 15) != (prevMinute / 15) ) && ( nowTime > 30000 ) )
      {
        DEBUG( "Scheudle start sat" );
        satStart();
      }
    }
    else
    {
      //night
      if ( ( nowHour != prevHour ) && ( nowTime > 30000 ) )
      {
        DEBUG( "Scheudle start sat" );
        satStart();
      }
    }
  }

  if ( satActive )
  {
    if ( !dispActive )
    {
      dispStart();
    }
  }

  if ( dispActive )
  {
    if ( !windActive )
    {
      windStart();
    }
    if ( !batActive )
    {
      batStart();
    }
  }

  /* controll power ON */
  if ( 1 )
  {
    if ( rtcActive || windActive || dispActive || satActive )
    {
      auxPowerOn();
    }
  }

  /* run other stuff */
  if ( batActive )
  {
    batRun();
  }
  if ( rtcActive )
  {
    rtcRun();
  }
  if ( windActive )
  {
    windRun();
  }

  if ( dispActive )
  {
    // run once a second or when button is pressed
    if ( stopSleep || ( prevSec != nowSec ) )
    {
      dispRun();
    }
    stopSleep = 0;
  }

  if ( satActive )
  {
    satRun();
  }

  prevHour = nowHour;
  prevMinute = nowMinute;
  prevSec = nowSec;


  /* controll power OFF */
  if ( 1 )
  {
    if ( ! ( rtcActive || windActive || dispActive || satActive ) )
    {
      auxPowerOff();
    }
  }

  if ( !satActive && !rtcActive && !windActive && !dispActive && !batActive )
  {
    //DEBUG( "Scheudle start deepSleep" );
    deepSleep( 60 - nowSec );
  }
}


ISR(WDT_vect) // interupt handler for watchdog time
{
}

void setupSleep()
{
  MCUSR &= ~(1 << WDRF); // don't reset on WDT

  WDTCSR |= (1 << WDCE) | (1 << WDE); // setup prescaler
  WDTCSR = 1 << WDP0 | 1 << WDP3; // set pre scaler to 4 seconds

  WDTCSR |= _BV(WDIE); // emable WDT interupt
}

void deepSleep( long t /* seconds */ )
{
  if ( t < 8 )
  {
    lightSleep(t);
    return;
  }
  DEBUG2( "deep sleep ", t );
  delay( 100 ); // give serial time to print

  while ( t >= 8 )
  {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    //DEBUG("power DOWN");
    //delay( 150 );

    sleep_enable();
    sleep_mode(); // go to sleep

    // we sleep here

    sleep_disable();
    power_all_enable();

    numDeepSleeps++;

    t = t - 8;
  }

  delay( 10 );
  //delay( 50 );
  //DEBUG("power UP");
}


void lightSleep(long t /* seconds*/)
{
  DEBUG2( "light sleep ", t );
  if ( t > 10 ) t = 10;

  stopSleep = 0;
  long time = t * 10;
  while ( !stopSleep && (time > 0) )
  {
    delay( 100 /*ms*/ );
    time--;
  }
  if ( stopSleep )
  {
    DEBUG( "interupt stopped deepSleep" );
  }
}


/****   Wind stuff  ****/
unsigned long windStartTime;
unsigned int  lastWindSpeedMPSx100 = 0;
unsigned int  maxWindSpeedMPSx100 = 0;
unsigned long sumWindSpeedMPSx100 = 0;
unsigned int  countWindSpeed = 0;


void windSetup()
{
  DEBUG( "in windSetup");

  windReset();
}

void windReset()
{
  lastWindSpeedMPSx100 = 0xFFFF;
  maxWindSpeedMPSx100 = 0;
  sumWindSpeedMPSx100 = 0;
  countWindSpeed = 0;
}

void windStart()
{
  if (windActive) return;

  windStartTime = nowTime;
  windActive = 1;
  lastWindSpeedMPSx100 = 0xFFFF; // correct for long ? TODO
}

void windRun()
{
  //DEBUG( "In windRun" );
  if ( nowTime < windStartTime + 1500 ) // takes 1.5 seconds to get a stable reading
  {
    return;
  }

  // sensor returns 0.4 v for 0 m/s wind and 2.0v for max wind of 32.4 m/s
  unsigned int val = analogRead(windPin); // max val is 1023 at 3.3 v
  //DEBUG2( "wind raw value = " , val );

  if ( 0 ) // fake wind
  {
    //val = 242; // this is about 15 knots
    val = prevMinute * 2 + 124;
  }

  if ( val < 100 )
  {
    lastWindSpeedMPSx100 = 0xFFFF;
  }
  else
  {
    if ( val < 124 )
    {
      val = 124;
    }

    val = val - 124;
    val = val * 13; // max val is 11,687
    val = val / 2;

    if ( lastWindSpeedMPSx100 ==  0xFFFF )
    {
      DEBUG2( "wind speed = " , val ); // oply print the first time for each winStart / stop
    }

    lastWindSpeedMPSx100 = val;
    if ( lastWindSpeedMPSx100 > maxWindSpeedMPSx100 )
    {
      maxWindSpeedMPSx100 = lastWindSpeedMPSx100;
    }
    sumWindSpeedMPSx100 += (unsigned long)lastWindSpeedMPSx100;
    countWindSpeed++;

  }

  windStop();
}

void windStop()
{
  if ( !dispActive )
  {
    windActive = 0;
  }
  //DEBUG2( "In windStop speed x100 = " , lastWindSpeedMPSx100 );
}

unsigned int windGetSpeedMPSx100()
{
  return lastWindSpeedMPSx100;
}


unsigned int windGetGustSpeedMPSx100()
{
  return maxWindSpeedMPSx100;
}


unsigned int windGetAvgSpeedMPSx100()
{
  unsigned long avg = sumWindSpeedMPSx100 / (unsigned long)countWindSpeed;
  return avg;
}


/****   RTC stuff  ****/
unsigned long rtcStartTime = 0;
unsigned long rtcStartSeconds; // wall clock time of startTime

void rtcSetup()
{
  DEBUG( "In rtcSetup");
  if ( disableRTC )
  {
    return;
  }
}

void rtcSetTime() {
  Wire.beginTransmission(rtcAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0x00)); // seconds (bit 7=0 is start)
  Wire.write(byte(0x443)); // minutes
  Wire.write(byte(0x22)); // hours (bit 6=0 is 24 hour mode)
  Wire.write(byte(0x07)); // day of week
  Wire.write(byte(0x06)); // day of month
  Wire.write(byte(0x08)); // month
  Wire.write(byte(0x14)); // year
  Wire.write(byte(0x00)); // control turn of square wave
  Wire.endTransmission();
}


void rtcStart()
{
  if (rtcActive) return;

  DEBUG( "In rtcStart");
  if ( disableRTC )
  {
    rtcActive = 0;
    return;
  }

  rtcStartTime = nowTime;
  rtcActive = 1;
}

byte bcdToDec( byte in )
{
  byte low = in & 0xF;
  byte high = (in >> 4) & 0xF;
  if ( (low > 9) || ( high > 9 ) )
  {
    return 0xFF;
  }

  byte v = high * 10 + low;
  return v;
}

byte rawTime1, rawTime2, rawTime3;// raw time btyes

void rtcRun()
{
  if ( disableRTC )
  {
    rtcStop();
    return;
  }

  //DEBUG( "In rtcRun 0");

  delay(10);

  byte h, m, s;
  if (1)
  {
    rtcGetTime( &h, &m, &s );
    DEBUG_NOCR( "rtcRun predict time = " );
    DEBUG_NOCR( h / 10 ); DEBUG_NOCR( h % 10 ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( m / 10 ); DEBUG_NOCR( m % 10 ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( s / 10 ); DEBUG( s % 10 );
  }

  rtcStartTime = nowTime;
  numDeepSleeps = 0;

  s = 0xFF;

  Wire.beginTransmission(rtcAddress);
  Wire.write(byte(0x00));
  Wire.endTransmission();

  //DEBUG( "In rtcRun 1");

  Wire.requestFrom(rtcAddress, 3); // num bytes to ready
  rawTime1 = Wire.read() ;
  rawTime2 = Wire.read();
  rawTime3 = Wire.read();

  //DEBUG( "In rtcRun 2");

  s = bcdToDec( rawTime1 & 0x7f ); // second - mask out CH bit
  m = bcdToDec( rawTime2 ); // minute
  h = bcdToDec( rawTime3 & 0x3f );  // hour - assume 24 hour mode

  //dayOfWeek  = bcdToDec( Wire.read() ); // dayOfWeek 1 to 7
  //dayOfMonth = bcdToDec( Wire.read() ); // dayOfMonbth 1 to 31
  //month      = bcdToDec( Wire.read() ); // month 1 to 12
  //year       = bcdToDec( Wire.read() ); // year will be 0 to 99

  if ( (s > 60) || ( m > 60 ) || ( h > 24 ) )
  {
    // got a bogus read of RTC - just go to fake it mode
    DEBUG("RTC Error - using FAKE TIME" );
    unsigned long t = millis(); //
    t = t / 1000; // sec
    s = (t % 60);
    t = t / 60; // now in min
    m = (t % 60);
    t = t / 60; //now in hour
    h = (t % 24);
  }

  if (1)
  {
    DEBUG_NOCR( "rtcRun real time = " );
    DEBUG_NOCR( h / 10 ); DEBUG_NOCR( h % 10 ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( m / 10 ); DEBUG_NOCR( m % 10 ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( s / 10 ); DEBUG( s % 10 );
  }

  rtcStartSeconds = (long)h * 3600 + (long)m * 60 + (long)s;

  rtcStop();
}

void rtcStop()
{
  rtcActive = 0;
  if ( disableRTC )
  {
    return;
  }
}

void rtcGetTime(byte* hour, byte* m, byte* sec)
{
  long deltaSeconds = (nowTime - rtcStartTime) / 1000  + (long)numDeepSleeps * 8  ; // TODO - what happens wrap ...
  unsigned long seconds = rtcStartSeconds + deltaSeconds;

  *sec = (seconds % 60);
  *m = ( (seconds / 60) % 60 );
  *hour = ( (seconds / 3600) % 24 );

  if ( 0 )
  {
    DEBUG_NOCR( "rtcGetTime = " );
    DEBUG_NOCR( *hour ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( *m ); DEBUG_NOCR( ":" );
    DEBUG( *sec );
  }
}

/*****  Batter Monitor Stuff  *******/

unsigned long batStartTime;
unsigned int lastBatVoltageX10 = 0;

void batSetup()
{
  DEBUG( "in batSetup" );
  lastBatVoltageX10 = 0xFFFF;

  batStart();
  batRun();
  batStop();

  DEBUG2( "in batSetup voltage x10 = " , lastBatVoltageX10 );
}

void batStart()
{
  if (batActive)
  {
    return;
  }

  batStartTime = nowTime;
  batActive = 1;
}

void batRun()
{
  //DEBUG( "In batRun" );

  delay( 100 ); // need some delay here or get readings of 1.4 v - this may be too much delay

  int v;
  v = analogRead( batSensePin );
  v = v * 30; // will overflow it scaled by value larger than this
  v = v / 145;
  v = v + 8; //  adjust for protection diode on input
  lastBatVoltageX10 = v;

  // note - reported 12.5 when actually was 12.6

  batStop();
}

void batStop()
{
  if ( !dispActive )
  {
    batActive = 0;
  }
  //DEBUG2( "in batStop voltage x10 = " , lastBatVoltageX10 );
}

unsigned int batGetVoltageX10()
{
  return lastBatVoltageX10;
}



/****   Display stuff  ****/


void dispSetup()
{
  DEBUG("start dispSetup");
  byte err;

  if (disableDisplay)
  {
    DEBUG("Display disabled");
    return;
  }

  Wire.beginTransmission(displayAddress);
  Wire.write( 0x7A );
  Wire.write(  0xFF ); // full brightness
  err = Wire.endTransmission();
  if (err)
  {
    DEBUG2( "distSetup i2c error = " , err );
  }

  DEBUG("done dispSetup");
}

void dispStart()
{
  if (disableDisplay)
  {
    dispActive = 0;
    return;
  }
  if (dispActive) return;

  // this runs in interup so don;t do much here
  dispStartTime = nowTime;
  dispActive = 1;
  dispItem = 0;
}


void dispRun()
{
  if (disableDisplay)
  {
    dispStop();
    return;
  }

  //DEBUG_NOCR2( "in dispRun " , dispItem );

  const long turnOffTime = 90000; // in ms
  if ( nowTime >= dispStartTime + turnOffTime )
  {
    dispStop();
    return;
  }

  switch ( dispItem )
  {
    case 99: // raw time butes
      {
        dispShow( rawTime2 >> 4, rawTime2 & 0xF ,  rawTime1 >> 4, rawTime1 & 0xF , -2 );
      }
      break;

    case 0: // wind knots
      {
        unsigned int w;
        w  = windGetSpeedMPSx100();
        if ( w == 0xFFFF )
        {
          dispShow(  '-', '-', '-', 'n', 0 );
        }
        else
        {
          w = w * 19; // convert to knotsx1000 TODO - can this overflow ? - more accureate converstion
          w = w / 100; // convert to 10ths
          dispShow( (w / 100) % 10, (w / 10) % 10,  (w) % 10 , 'n' , 2 );
        }
      }
      break;

    case 1: // time
      {
        byte hour,  m,  sec;
        rtcGetTime( &hour, &m, &sec);
        if ( 0 )
        {
          dispShow( m / 10, m % 10 , sec / 10, sec % 10, -2 );
        }
        else
        {
          dispShow( hour / 10, hour % 10 , m / 10, m % 10, -2 );
        }
      }
      break;

    case 2: // voltage
      {
        unsigned int v;
        v = batGetVoltageX10();
        dispShow( ' ', (v / 100) % 10, (v / 10) % 10, v % 10, 1 );
      }
      break;

    case 3: // time since last sat tx
      {
        long delta = ( nowTime - satGetLastTxTime() ) / 1000; // TODO - likely wrong due to deep sleep and milis not advancing
        delta = delta / 60;
        dispShow( '-', (delta / 100) % 10, (delta / 10) % 10, delta % 10 , 0 );
      }
      break;

    case 4: // sat error
      {
        byte e;
        e = satGetError();
        switch (e ) {
          case 0:
            dispShow( '-', '-', '-', '-' , -1 );
            break;
          case 0xFF:
            dispShow( 'E', 'r', 'r', ' ' , -1 );
            break;
          default:
            dispShow( 'E', ' ', (e / 16) % 16, e % 16 , -1 );
            break;
        }
      }
      break;

  }
}


void dispNext()
{
  if (disableDisplay)
  {
    return;
  }

  // increment to display next Item
  dispStartTime = nowTime;
  dispItem++;
  if ( dispItem > 4 )
  {
    dispItem = 0;
  }
}


void dispShow( byte a, byte b, byte c, byte d, int n )
{
  if (disableDisplay)
  {
    return;
  }

  // DEBUG2( " n=", n );

  if (0)
  {
    //  debug of DISPLAY
    char data[8];
    data[0] = ( a <= 0x0F ) ? '0' + a : a ;
    data[1] = ( n == 3 ) ? '.' : ' ' ;
    data[2] = ( b <= 0x0F ) ? '0' + b : b ;
    data[3] = ( n == 2 ) ? '.' :  (( n == -2 ) ? ':' : ' ') ;
    data[4] = ( c <= 0x0F ) ? '0' + c : c ;
    data[5] = ( n == 1 ) ? '.' :  (( n == -1 ) ? 0x27 : ' ') ;
    data[6] = ( d <= 0x0F ) ? '0' + d : d ;
    data[7] = 0;

    DEBUG2( " DISPLAY ", data );
  }

  // display a,b,c,d with decimal in n'th postion -2 for colon
  Wire.beginTransmission(displayAddress);
  Wire.write( 0x76 ); // clear
  Wire.write( a );
  Wire.write( b );
  Wire.write( c );
  Wire.write( d );
  switch ( n ) {
    case 0:
      break;
    case 1:
      Wire.write( 0x77 );  Wire.write(  0x04 );
      break;
    case 2:
      Wire.write( 0x77 );  Wire.write(  0x02 );
      break;
    case 3:
      Wire.write( 0x77 );  Wire.write(  0x01 ); // turn on decimal
      break;
    case -2:
      Wire.write( 0x77 );  Wire.write(  0x10 ); // turn on colon
      break;
    case -1:
      Wire.write( 0x77 );  Wire.write(  0x20 ); // turn on Apostrophe
      break;
  }
  Wire.endTransmission();
}


void dispStop()
{
  dispActive = 0;

  if (disableDisplay)
  {
    return;
  }

  dispShow( ' ', ' ', ' ', ' ', 0 );
}


/****   SAT stuff  ****/

unsigned long satStartTime;
unsigned long satLastTxTime;
byte satTxCount;
byte satLastErr;

void satSetup()
{
  satActive = 0;

  satSerial.begin( 19200 );
  sat.attachConsole(Serial);
  sat.attachDiags(Serial);
  sat.setPowerProfile(0);

  if ( !disableSat )
  {
    if ( !sat.isAsleep() )
    {
      sat.sleep();
    }
  }
}


void satStart()
{
  if ( satActive ) return;

  satActive = 1;
  satStartTime = nowTime;
  satLastErr = 0xFF;
  satTxCount = 0;
}


void satRun()
{
  DEBUG( "in satRun" );

  if ( !disableSat )
  {
    if ( sat.isAsleep() )
    {
      sat.begin();
    }
  }

  int sigQuality = 0;
  int err;

  if (!disableSat)
  {
    err = sat.getSignalQuality( sigQuality );
    if (err != 0)
    {
      DEBUG2( "getSignalQuality failed err=" , err );
      satLastErr = err;
      satStop();
      return;
    }
    DEBUG2( "Sat signal quality = " , sigQuality );

    if ( sigQuality < 2 )
    {
      // wait 5 minutes for better sigQuality
      if ( nowTime < satStartTime + (long)300000 ) // wait up to 5 min for good sat signal
      {
        DEBUG( "Waiting for better sat signal" );
        digitalWrite(ledPin, LOW);
        return;
      }
      else
      {
        DEBUG( "Gave up Waiting for better sat signal" );
      }
    }
  }

  satTxCount++;

  String satMsg;
  satMsg.reserve(124); // set size (currenly use 119 + termination)
  unsigned int w = windGetAvgSpeedMPSx100();
  unsigned int g = windGetGustSpeedMPSx100();
  unsigned int v = batGetVoltageX10();

  satMsg = "{\"bn\":\"RB8920/v1/\","; // TODO - move hardcode name to top 
  //satMsg += "\"ver\":1,";
  satMsg += "\"bu\":\"m/s\",";
  // addding a base time makes this too large
  satMsg += "\"e\":[";

  satMsg += "{\"n\":\"battery\",\"u\":\"V\",\"v\":";
  satMsg += v / 10 ;
  satMsg += ".";
  satMsg += v % 10 ;
  if (1) // TODO - sat signal level + retry hack
  {
    satMsg += 0;
    satMsg += sigQuality % 10 ;
    satMsg += satTxCount % 10 ;
  }
  satMsg += "},";

  satMsg += "{\"n\":\"gust\",\"v\":";
  satMsg += g / 100 ;
  satMsg += ".";
  satMsg += (g / 10) % 10 ;
  if ( g % 10 != 0 )
  {
    satMsg += g % 10 ;
  }
  satMsg += "},";

  satMsg += "{\"n\":\"wind\",\"v\":";
  satMsg += w / 100 ;
  satMsg += ".";
  satMsg += (w / 10) % 10 ;
  if ( w % 10 != 0 )
  {
    satMsg += w % 10 ;
  }
  satMsg += "}";

  satMsg += "]}";
  DEBUG2( "satMsg=", satMsg );
  DEBUG2( "satMsg len=", satMsg.length() );

  if ( !disableSat )
  {
    err = sat.sendSBDText( satMsg.c_str() );
    if (err != 0)
    {
      DEBUG2( "sat sendSBDText failed: ", err );
      satLastErr = err;
    }
    else
    {
      DEBUG("Sat Transmit OK **********************************************");
      satLastErr = 0;
      satLastTxTime = millis(); // TODO - this will compute wrong with sleeps - need to set with RTC
      satStop();
      windReset();
    }
  }
  else
  {
    satLastErr = 0;
    satLastTxTime = millis(); // TODO - this will compute wrong with sleeps - need to set with RTC
    satStop();
  }

  if ( satTxCount >= 4 ) // TODO - adjust - retry transmition 4 times
  {
    satStop();
  }


  digitalWrite(ledPin, LOW);
}


void satStop()
{
  if ( !disableSat )
  {
    if ( !sat.isAsleep() )
    {
      sat.sleep();
    }
  }

  digitalWrite(ledPin, LOW);
  satActive = 0;
}


byte satGetError()
{
  return satLastErr;
}


unsigned long satGetLastTxTime()
{
  return satLastTxTime; // TODO - prob wrong where used since based on milis with no deep sleep adjustment
}


bool ISBDCallback()
{
  static unsigned long prevTime = 0;
  unsigned long now = millis();

  digitalWrite(ledPin, (now / 200) % 3 == 0 ? HIGH : LOW);

  if ( now / 1000 != prevTime / 1000 )
  {
    prevTime = now;

    long s = ( now - satStartTime ) / 1000;

    dispShow( 'S', ' ' , (s / 10) % 10, s % 10 , -1 );
  }

  return true;
}













