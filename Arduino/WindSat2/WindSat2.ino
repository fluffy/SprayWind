

#include <SoftwareSerial.h>
#include <IridiumSBD.h>
#include <Wire.h>


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


#define DEBUG( X ) Serial.println( X );
#define DEBUG_NOCR( X ) Serial.print( X );
#define DEBUG2( X, Y ) Serial.print( X ); Serial.println( Y );
#define DEBUG_NOCR2( X,Y ) Serial.print( X) ; Serial.print( Y );

byte satActive = 0;
byte rtcActive = 0;
byte windActive = 0;
byte batActive = 0;
volatile byte dispActive = 0;

/****   Display stuff  ****/
volatile unsigned long dispStartTime = 0;
volatile byte dispItem = 0;


void setup()
{
  Serial.begin(9600);
  //Serial.begin(19200);

  DEBUG("Setup");

  pwrSetup();
  btnSetup();

  Wire.begin();
  analogReference( DEFAULT ); // 3.3 V


  if (0)
  {
    rtcSetTime();
  }

  satSetup();
  rtcSetup();
  windSetup();
  batSetup();
  dispSetup();

  rtcStart();
  windStart();
  batStart();

  if (0)
  {
    while ( rtcActive ) // wait to get a valid time before really starting
    {
      rtcRun();
    }
  }

  dispStart(); // don't start till after have a time or rtc remains active until display is not
}


void loop()
{
  DEBUG_NOCR(".");

  //delay( 500 /*ms*/ ); // TODO remove
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
  satPowerOn();
}

void auxPowerOn()
{
  if ( auxPowerStatus )
  {
    return;
  }
  digitalWrite( auxPwrEnablePin, HIGH );
  auxPowerStatus = 1;
}

void auxPowerOff()
{
  if ( !auxPowerStatus )
  {
    return;
  }
  digitalWrite( auxPwrEnablePin, LOW );
  auxPowerStatus = 0;
}

void satPowerOn()
{
}

void satPowerOff()
{
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


byte prevMinute = -1;
byte prevSec = -1;

void runSched() {
  //DEBUG( "In runSched" );
  //DEBUG_NOCR( "." );

  nowTime = millis();
  unsigned long t = (nowTime / 60000) % 60;
  byte nowMinute, nowSec, nowHour;
  rtcGetTime( &nowHour, &nowMinute, &nowSec);

  //DEBUG2( "nowMinute  = ", nowMinute );
  //DEBUG2( "prevMinute = ", prevMinute );

  if ( nowMinute != prevMinute )
  {
    DEBUG( "Scheudle start wind" );
    windStart();
  }

  if ( nowMinute != prevMinute ) // todo less often
  {
    DEBUG( "Scheudle start bat" );
    batStart();
  }

  if ( ((nowMinute + 1) / 5) != ((prevMinute + 1) / 5) ) // ever 5 min, but 1 min before satStart
  {
    DEBUG( "Scheudle start rtc" );
    rtcStart();
  }

  // TODO - add hour check here and schedule
  if ( (nowMinute / 15) != (prevMinute / 15) )
  {
    DEBUG( "Scheudle start sat" );
    satStart();
  }

  /* controll power */
  if ( 0 ) // TODO
  {
    if ( rtcActive || windActive || dispActive || satActive )
    {
      auxPowerOn();
    }
    else
    {
      auxPowerOff();
    }
    if ( satActive )
    {
      satPowerOn();
    }
    else
    {
      satPowerOff();
    }
  }

  /* run other stuff */
  if ( rtcActive )
  {
    rtcRun();
  }
  if ( windActive )
  {
    windRun();
  }
  if ( batActive )
  {
    batRun();
  }
  if ( satActive )
  {
    satRun();
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

    // run once a second or when button is pressed
    if ( stopSleep || ( prevSec != nowSec ) )
    {
      dispRun();
    }
    stopSleep = 0;
  }

  prevMinute = nowMinute;
  prevSec = nowSec;

  if ( !satActive && !rtcActive && !windActive && !dispActive )
  {
    //DEBUG( "Scheudle start deepSleep" );
    deepSleep( 60 - nowSec );
  }


}

void deepSleep(long t/* seconds*/)
{
  if ( t > 10 ) t = 10; // TODO - consider remove

  DEBUG2( "in Deep sleep ", t );
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
long lastWindSpeedMPSx100 = 0;

void windSetup()
{
  DEBUG( "in windSetup");
}

void windStart()
{
  windStartTime = nowTime;
  windActive = 1;
  lastWindSpeedMPSx100 = 0xFFFF;
}

void windRun()
{
  //DEBUG( "In windRun" );


  delay( 5 ); // TODO measure time to stable from power on

  //if ( nowTime < windStartTime + 100 )
  //{
  //  return;
  //}

  // sensor returns 0.4 v for 0 m/s wind and 2.0v for max wind of 32.4 m/s
  int val = analogRead(windPin);
  //DEBUG2( "wind raw value = " , val );

  //val = 242; // TODO - this is about 15 knots

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
    val = val * 13;
    val = val / 2;

    lastWindSpeedMPSx100 = val;
  }

  windStop();
}

void windStop()
{
  if ( !dispActive )
  {
    windActive = 0;
  }
}

long windGetSpeedMPSx100()
{
  return lastWindSpeedMPSx100;
}

/****   RTC stuff  ****/
unsigned long rtcStartTime = 0;
unsigned long rtcStartSeconds; // wall clock time of startTime

void rtcSetup()
{
  DEBUG( "In rtcSetup");
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
  DEBUG( "In rtcStart");
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
  //DEBUG( "In rtcRun");

  delay( 2 ); // mesure time from power on to usable

  // if ( nowTime < rtcStartTime + 5 )
  // {
  //   return;
  // }

  byte h, m, s;
  rtcStartTime = nowTime;


  s = 0xFF;

  if (1) // TODO enable
  {
    Wire.beginTransmission(rtcAddress);
    Wire.write(byte(0x00));
    Wire.endTransmission();

    Wire.requestFrom(rtcAddress, 3); // num bytes to ready
    rawTime1 = Wire.read() ;
    rawTime2 = Wire.read();
    rawTime3 = Wire.read();

    s = bcdToDec( rawTime1 & 0x7f ); // second - mask out CH bit
    m = bcdToDec( rawTime2 ); // minute
    h = bcdToDec( rawTime3 & 0x3f );  // hour - assume 24 hour mode

    //dayOfWeek  = bcdToDec( Wire.read() ); // dayOfWeek 1 to 7
    //dayOfMonth = bcdToDec( Wire.read() ); // dayOfMonbth 1 to 31
    //month      = bcdToDec( Wire.read() ); // month 1 to 12
    //year       = bcdToDec( Wire.read() ); // year will be 0 to 99
  }

  if ( (s > 60) || ( m > 60 ) || ( h > 24 ) )
  {
    if (0) // todo
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
    else
    {
      h = m = s = 6; // TODO remove
    }
  }

  if (1) // TODO
  {
    DEBUG_NOCR( "rtcRun real time = " );
    DEBUG_NOCR( h ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( m ); DEBUG_NOCR( ":" );
    DEBUG( s );
  }

  rtcStartSeconds = (long)h * 3600 + (long)m * 60 + (long)s;

  rtcStop();
}

void rtcStop()
{
  //DEBUG( "In rtcStop");
  //if ( !dispActive ) // TODO decide
  {
    rtcActive = 0;
  }
}

void rtcGetTime(byte* hour, byte* m, byte* sec)
{
  long deltaSeconds = (nowTime - rtcStartTime) / 1000; // TODO - what happens wrap ...
  unsigned long seconds = rtcStartSeconds + deltaSeconds;

  //seconds = rtcStartSeconds;  // TODO REMOVE
  //DEBUG2( "seconds=" , seconds );

  *sec = (seconds % 60);
  *m = ( (seconds / 60) % 60 );
  *hour = ( (seconds / 3600) % 24 );

  if ( 0 ) // TODO
  {
    DEBUG_NOCR( "rtcGetTime = " );
    DEBUG_NOCR( *hour ); DEBUG_NOCR( ":" );
    DEBUG_NOCR( *m ); DEBUG_NOCR( ":" );
    DEBUG( *sec );
  }
}

/*****  Batter Monitor Stuff  *******/

unsigned long batStartTime;
int lastBatVoltageX10 = 0;

void batSetup()
{
  DEBUG( "in batSetup" );
  lastBatVoltageX10 = 0xFFFF;
}

void batStart()
{
  batStartTime = nowTime;
  batActive = 1;
}

void batRun()
{
  //DEBUG( "In batRun" );

  delay( 2 ); // how long to charge cap from power on
  //if ( nowTime < batStartTime + 10 )
  //{
  //  return;
  //}

  int v = analogRead( batSensePin );
  v = v * 30; // will overflow it scale by value larger than this
  v = v / 145;
  lastBatVoltageX10 = v;

  batStop();
}

void batStop()
{
  if ( !dispActive )
  {
    batActive = 0;
  }
}

int batGetVoltageX10()
{
  return lastBatVoltageX10;
}



/****   Display stuff  ****/


void dispSetup()
{
  DEBUG("start dispSetup");
  if ( 1 ) // TODO
  {
    Wire.beginTransmission(displayAddress);
    Wire.write( 0x7A );  Wire.write(  0xFF ); // full brightness
    Wire.endTransmission();
  }
  DEBUG("done dispSetup");
}

void dispStart()
{
  // this runs in interup so don;t do much here
  dispStartTime = nowTime;
  dispActive = 1;
  dispItem = 0;
}

void dispRun()
{
  DEBUG_NOCR2( "in dispRun " , dispItem );

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
        int w;
        w  = windGetSpeedMPSx100();
        if ( w == 0xFFFF )
        {
          dispShow(  'E', 'r', 'r', 'S', -1 );
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
        if ( 1 ) // TODO
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
        long v;
        v = batGetVoltageX10();
        dispShow( ' ', (v / 100) % 10, (v / 10) % 10, v % 10, 1 );
      }
      break;

    case 3: // time since lat sat tx
      {
        //dispShow( 'n', 'o', ' ', ' ' , 0 );
        dispShow( '-', 0, 0, ' ' , 0 );
      }
      break;

    case 4: // sat error
      {
        byte e;
        e = satGetError();
        switch (e ) {
          case 0:
            dispShow( 'O', 'K', ' ', ' ' , 0 );
            break;
          case 0xFF:
            dispShow( 'E', 'r', 'r', ' ' , 0 );
            break;
          default:
            dispShow( 'E', ' ', (e / 16) % 16, e % 16 , 0 );
            break;
        }
      }
      break;

  }
}

void dispNext()
{
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
  // DEBUG2( " n=", n );

  if (1 )
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
  dispShow( ' ', ' ', ' ', ' ', 0 );
}


/****   SAT stuff  ****/

unsigned long satStartTime;
byte satLastErr;

void satSetup()
{
  satActive = 0;

  satSerial.begin( 19200 );

  sat.attachConsole(Serial);
  sat.attachDiags(Serial);

  sat.setPowerProfile(0);

}

void satStart()
{
  satActive = 1;
  satStartTime = nowTime;
  satLastErr = 0xFF;

  sat.begin();
}

void satRun()
{
  DEBUG( "in satRun" );

  int sigQuality ;
  int err;

  err = sat.getSignalQuality(sigQuality);
  if (err != 0)
  {
    DEBUG2( "getSignalQuality failed err=" , err );
    satLastErr = err;
    satStop();
    return;
  }

  DEBUG2( "Sat signal quality = " , sigQuality );

  err = sat.sendSBDText("Hello, world!");
  if (err != 0)
  {
    DEBUG2( "sat sendSBDText failed: ", err );
    satLastErr = err;
    satStop();
    return;
  }

  DEBUG("Sat Transmit OK **********************************************");


  DEBUG2( "sat msg's left = " ,  sat.getWaitingMessageCount() );


  satStop(); // TODO
}

void satStop()
{
  sat.sleep();
  satActive = 0;
}

byte satGetError()
{
  return satLastErr;
}



bool ISBDCallback()
{
  digitalWrite(ledPin, (millis() / 500) % 2 == 1 ? HIGH : LOW);
  return true;
}













