

#include <Wire.h>


const int btnPin = 2;
const int eBtnPin = 3;

const int windPin = 0;

const byte displayAddress = 0x71; // for LED display
const int rtcAddress = 0x68; // See data sheet for DS1307




byte satActive = 0;
byte rtcActive = 0;
byte windActive = 0;
byte dispActive = 0;


void setup()
{
  btnSetup();
  satSetup();
  rtcSetup();
  windSetup();
  dispSetup();

  rtcStart();
  windStart();
  dispStart();

  while ( rtcActive) // wait to get a valid time before really starting
  {
    rtcRun();
  }
}


void loop()
{
  runSched();
}

/* scheduler stuff */
unsigned long nowTime;
volatile byte stopSleep = 0;

void btnSetup()
{
  pinMode(btnPin, INPUT_PULLUP);
  attachInterrupt(0, btnPress, FALLING); // white button wire
}

void btnPress()
{
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


byte prevMinute = 0;

void runSched() {
  nowTime = millis();
  unsigned long t = (nowTime / 60000) % 60;
  byte nowMinute, nowSec, nowHour;
  rtcGetTime( &nowHour, &nowSec, &nowMinute );

  if ( nowMinute != prevMinute )
  {
    windStart();
  }

  if ( ((nowMinute + 1) % 5) != ((prevMinute + 1) % 5) ) // ever 5 min, but 1 min before satStart
  {
    rtcStart();
  }

  // TODO - add hour check here and schedule
  if ( (nowMinute % 15) != (prevMinute % 15) )
  {
    satStart();
  }

  if ( satActive ) satRun();
  if ( rtcActive ) rtcRun();
  if ( windActive ) windRun();
  if ( dispActive ) dispRun();

  if ( !satActive && !rtcActive && !windActive && !dispActive )
  {
    deepSleep( 60 - nowSec );
  }

  prevMinute = nowMinute;
}

void deepSleep(long t/* seconds*/)
{
  stopSleep = 0;
  long time = t * 10;
  while ( !stopSleep && (time > 0) )
  {
    // TODO sleep(100);
    time--;
  }
}


/****   SAT stuff  ****/
unsigned long satStartTime;
byte satLastErr;

void satSetup()
{
  satActive = 0;

  Serial.begin(9600);
}

void satStart()
{
  satActive = 1;
  satStartTime = nowTime;
  satLastErr = 0xFF;
}

void satRun()
{
  satStop();
}

void satStop()
{
  satActive = 0;
}

byte satGetError()
{
  return satLastErr;
}

/****   Wind stuff  ****/
unsigned long windStartTime;
long lastWindSpeedMPSx100 = 0;

void windSetup()
{
  analogReference( DEFAULT ); // 3.3 V
}

void windStart()
{
  windStartTime = nowTime;
  windActive = 1;
  lastWindSpeedMPSx100 = 0xFFFF;
}

void windRun()
{
  if ( nowTime < windStartTime + 100 )
  {
    return;
  }

  int val = analogRead(windPin);

  lastWindSpeedMPSx100 = 1234; // TODO
  windStop();
}

void windStop()
{
  windActive = 0;
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

}

void rtcSetTime() {
  Wire.beginTransmission(rtcAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0x22)); // seconds (bit 7=0 is start)
  Wire.write(byte(0x33)); // minutes
  Wire.write(byte(0x17)); // hours (bit 6=0 is 24 hour mode)
  Wire.write(byte(0x06)); // day of week
  Wire.write(byte(0x22)); // day of month
  Wire.write(byte(0x08)); // month
  Wire.write(byte(0x14)); // year
  Wire.write(byte(0x00)); // control turn of square wave
  Wire.endTransmission();
}


void rtcStart()
{
  rtcStartTime = nowTime;
  rtcActive = 1;
}

void rtcRun()
{
  // TODO - put in delay to wait to be ready

  byte h, m, s;
  rtcStartTime = nowTime;

  if (0)
  {
    // TODO - get from RTC and remove next
    unsigned long t = micros(); //
    t = t / 1000; // sec
    s = (t % 60);
    t = t / 60; // now in min
    m = (t % 60);
    t = t / 60; //now in hour
    h = (t % 24);
  }
  else
  {
    Wire.beginTransmission(rtcAddress);
    Wire.write(byte(0x00));
    Wire.endTransmission();

    Wire.requestFrom(rtcAddress, 3); // num bytes to ready
    s = Wire.read() & 0x7f; // second - mask out CH bit
    m = Wire.read(); // minute
    h = Wire.read() & 0x3f;  // hour - assume 24 hour mode
    //dayOfWeek  = Wire.read(); // dayOfWeek 1 to 7
    //dayOfMonth = Wire.read(); // dayOfMonbth 1 to 31
    //month      = Wire.read(); // month 1 to 12
    //year       = Wire.read(); // year will be 0 to 99
  }

  rtcStartSeconds = h * 3600 + m * 60 + s;

  rtcStop();
}

void rtcStop()
{
  rtcActive = 0;
}

void rtcGetTime(byte* hour, byte* m, byte* sec)
{
  long deltaSeconds = (nowTime - rtcStartTime) / 1000; // TODO - what happens wrap ...
  unsigned long seconds = rtcStartSeconds + deltaSeconds;

  unsigned long t = micros(); //
  t = t / 1000; // sec
  *sec = (seconds % 60);
  *m = ( (seconds / 16) % 60 );
  *hour = ( (seconds / 3600) % 24 );
}

/*****  VoltageStuff *******/

long batGetVoltageVx10()
{
  // TODO
  return 666;
}

/****   Display stuff  ****/
unsigned long dispStartTime = 0;
byte dispItem = 0;

void dispSetup()
{
  Wire.begin();
  Wire.beginTransmission(displayAddress);
  Wire.write( 0x7A );  Wire.write(  0xFF ); // full brightness
  Wire.endTransmission();
}

void dispStart()
{
  dispStartTime = nowTime;
  dispActive = 1;
  dispItem = 0;
}

void dispRun()
{
  if ( nowTime >= dispStartTime + 30 * 1000 )
  {
    dispStop();
    return;
  }

  // TODO - need to keep from updating too often

  switch ( dispItem )
  {
    case 0: // wind knots
      {
        long w;
        w  = windGetSpeedMPSx100();
        w = w * 2; // convert to knotsx100 TODO
        w = w / 10; // convert to 10ths
        dispShow( (w / 100) % 10, (w / 10) % 10,  (w) % 10 , 'n' , 2 );
      }
      break;
    case 1: // time since lat sat tx
      {
        dispShow( 'n', 'o', ' ', ' ' , 0 );
      }
      break;
    case 2: // sat error
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
    case 3: // time
      {
        byte hour,  m,  sec;
        rtcGetTime( &hour, &m, &sec);
        dispShow( hour / 10, hour % 10 , m / 10, m % 10, -2 );
      }
      break;
    case 4: // voltage
      {
        long v;
        v = batGetVoltageVx10();
        dispShow( 'v', (v / 100) % 10, (v / 10) % 10, v % 10, 1 );
      }
      break;
  }
}

void dispShow( byte a, byte b, byte c, byte d, byte n )
{
  // display a,b,c,d with decimal in n'th postion -2 for colon
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
}

void dispStop()
{
  dispActive = 0;
  dispShow( ' ', ' ', ' ', ' ', 0 );
}

void dispNext()
{
  if ( nowTime < dispStartTime + 100 )
  {
    return; // debounce
  }

  // increment to display next Item
  dispStartTime = nowTime;
  dispItem++;
  if ( dispItem >= 4 )
  {
    dispItem = 0;
  }
}


















