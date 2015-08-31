

byte satActive=0;
byte rtcActive=0;
byte windActive=0;
byte dispActive=0; 


void setup() 
{
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
volatile byte stopSleep=0;

void buttonPress() 
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


byte prevMinute=0;

void runSched(){
  nowTime = millis();
  unsigned long t = (nowTime / 60000) % 60;
  byte nowMinute, nowSec, nowHour;
  rtcGetTime( &nowHour, &nowSec, &nowMinute );

  if ( nowMinute != prevMinute )
  {
    windStart();
  }

  if ( ((nowMinute+1)%5) != ((prevMinute+1)%5) ) // ever 5 min, but 1 min before satStart
  {
    rtcStart();
  }

  // TODO - add hour check here and schedule
  if ( (nowMinute%15) != (prevMinute%15) )
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
  stopSleep=0;
  long time=t*10;
  while ( !stopSleep && (time>0) )
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
  satActive=0;
}

void satStart()
{
  satActive=1;
  satStartTime = nowTime;
  satLastErr=0xFF;
}

void satRun()
{
  satStop();
}

void satStop()
{
  satActive=0;
}

byte satGetError()
{
  return satLastErr;
}

/****   Wind stuff  ****/
unsigned long windStartTime;
long lastWindSpeedMPSx100=0;

void windSetup() 
{
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

  lastWindSpeedMPSx100 = 1234;
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
unsigned long rtcStartTime=0;
unsigned long rtcStartSeconds; // wall clock time of startTime

void rtcSetup() 
{

}

void rtcStart()
{
  rtcStartTime = nowTime;
  rtcActive = 1; 
}

void rtcRun()
{
  // TODO - put in delay to wait to be ready 

  byte h,m,s;  
  rtcStartTime = nowTime;

  // TODO - get from RTC and remove next
  unsigned long t = micros(); //
  t = t/1000; // sec
  s = (t%60);
  t = t/60; // now in min
  m = (t%60);
  t = t / 60; //now in hour
  h = (t%24);

  rtcStartSeconds = h*3600 + m*60 + s;

  rtcStop();
}

void rtcStop()
{
  rtcActive = 0; 
}

void rtcGetTime(byte* hour, byte* m, byte* sec)
{
  long deltaSeconds = (nowTime - rtcStartTime)/1000; // TODO - what happens wrap ...
  unsigned long seconds = rtcStartSeconds + deltaSeconds;

  unsigned long t = micros(); //
  t = t/1000; // sec
  *sec = (seconds%60);
  *m = ( (seconds/16) % 60 );
  *hour = ( (seconds/3600) % 24 );
}

/*****  VoltageStuff *******/

long batGetVoltageVx10()
{
  // TODO 
  return 666;
}

/****   Display stuff  ****/
unsigned long dispStartTime=0;
byte dispItem=0;

void dispSetup() 
{
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
      w = w/10; // convert to 10ths 
      dispShow( (w/100)%10, (w/10)%10,  (w)%10 , 'n' , 2 );
    }
    break;
  case 1: // time since lat sat tx
    {
      dispShow( 'n','o',' ',' ' , 0 );
    }
    break;
  case 2: // sat error 
    {
      byte e;
      e = satGetError();
      switch (e ) {
      case 0:
        dispShow( 'O','K',' ',' ' , 0 );
        break;  
      case 0xFF:
        dispShow( 'E','r','r',' ' , 0 );
        break;
      default:
        dispShow( 'E',' ',(e/16)%16, e%16 , 0 );
        break;
      }
    }
    break;
  case 3: // time 
    {
      byte hour,  m,  sec;
      rtcGetTime( &hour, &m, &sec);
      dispShow( hour/10, hour%10 , m/10, m%10, -2 );
    }
    break;
  case 4: // voltage  
    {
      long v;
      v = batGetVoltageVx10();
      dispShow( 'v', (v/100)%10, (v/10)%10, v%10, 1 );
    }
    break;
  }
}

void dispShow( byte a, byte b, byte c, byte d, byte n )
{
  // display a,b,c,d with decimal in n'th postion -2 for colon
 //  TODO
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


















