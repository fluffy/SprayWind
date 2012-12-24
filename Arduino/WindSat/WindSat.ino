/* 
 Copyright (c) 2012, Cullen Jennings <fluffy@iii.ca>
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met: 
 
 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer. 
 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution. 
 
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
 
 
 Example program to connect up SPOT Connect to Arduino using the SparkFun Run SPOT Run board. 
 
 The source code repository for this project is at: TODO 
 
 Using this Library:
 
 You have to set up a soft or hardware serial interface to the SPOT and it must be 115200 baud. 
 The SatConnect calss also takes the two pins to controll the power and on button to the Run SPOT Run board. 
 
 begin and end method turn the power on and off on the spot 
 write sends a message. It must be shorter than 41 bytes 
 
 you can use ready to find out when it is OK to do a write 
 
 you can use state to track the transmition of a message which takes aproximately 20 minutes 
 */

#include <Serial.h>
#define ONEWIRE_CRC16 0
#define ONEWIRE_CRC8_TABLE 0
#include <OneWire.h>

#define DEBUG( X ) Serial.print( "  " ); Serial.println( X ); delay( 20 );
#define DEBUG2( X, Y ) Serial.print( "  " ); Serial.print( X ); Serial.println( Y ); delay( 20 );

const int windPin = 2;     // the number of the wind speed pin - must be 2 for interupt 0 
const int batteryPin = 1;  // analog pin for battery voltage sense 
const int spotPowerPin = 6; /* PwrPin - turns on power to SPOT device */

const int spotOnPin = 7;  /* controlls the "on" button on the SPOT to turn the device on and off  */




// ========================================================================
// Stuff for Spot Connect Satalite code 

class SatConnect
{
public:
  SatConnect( Stream& pSerial, int pPwrPin, int pOnPin ); 

  enum SatConnectState 
  { 
    off,  poweringUp, readyToSend, haveMsgToSend, findingPostion, sending, sentOnce, sentTwice, errorNoGPS, errorNoSpot
  };

  bool ready();
  SatConnectState state();
  void begin();
  void end();

  void write( char* msg, int len );

private:
  const int pwrPin;
  const int onPin;
  Stream& serial;
  SatConnect::SatConnectState s; 
};


SatConnect::SatConnect( Stream& pSerial, int pPwrPin, int pOnPin ): 
serial(pSerial), pwrPin(pPwrPin), onPin(pOnPin)
{
  pinMode(pwrPin,OUTPUT);
  pinMode(onPin,OUTPUT);
  digitalWrite( pwrPin, LOW ); // turn off power to SPOT
  digitalWrite( pwrPin, HIGH ); // Leave button not pushed 
  s = SatConnect::off;
}


bool SatConnect::ready()
{
  if ( state() == SatConnect::readyToSend ) 
  {
    return true;
  }
  return false;
}


SatConnect::SatConnectState SatConnect::state()
{
  static unsigned long satLastTime=0;

  if ( s == SatConnect::off )
  {
    return s;
  }

  if ( s == SatConnect::errorNoSpot )
  {
    return s;
  }

  unsigned long now =  millis();
  if ( now < satLastTime + 500 ) // if state if less than 500 ms old, don't check for new state 
  { 
    return s;
  }
  satLastTime = now;

  // flush out any old stuff in serial buffer   
  unsigned long lastTime = millis();
  uint8_t c;
  do
  {
    if ( serial.available() )
    {
      c = serial.read();
      lastTime = millis();
      Serial.print("Flushing extra data: "); 
      Serial.println( c , HEX ); 
    }
  } 
  while ( millis() < lastTime + 150 ); // no data for 100 ms 

  static uint8_t statReq[3] = {  
    0xAA, 0x03, 0x52                               };
  serial.write( statReq, sizeof( statReq ) );

  lastTime = millis();
  uint8_t buf[ 0x30 ];
  buf[0] = 0;
  buf[1] = sizeof(buf);
  buf[7] = 0;
  buf[19]= 0;
  int i=0;
  do
  {
    if ( serial.available() )
    {
      buf[i++] = serial.read();
      lastTime = millis();
    }
    if (buf[0] != 0xAA) // keep reading until find the 0xAA start of message byte 
    {
      i=0;
    }
    if ( i == buf[1] )
    {
      //Serial.println( "Read full packet" );
      break;
    }
  } 
  while ( (millis() < lastTime + 150) && ( i < sizeof(buf) ) ); // 150 ms timeout after last char read 

  if ( (i != buf[1]) && (i+2 != buf[1]) ) // for some reason always getting 2 bytes short on some firware
  {
    int timeout = millis()-lastTime;
    Serial.println( "Problem reading packet buf" );
    Serial.print( "buf[1]=" ); 
    Serial.println( buf[1] );
    Serial.print( "i=" ); 
    Serial.println( i );
    Serial.print( "time out=" ); 
    Serial.println( timeout );
  }

#if 1
  //Serial.print( "i= 0x" ); 
  //Serial.println( i , HEX );
  Serial.print( "SPOT Status: " );
  for ( int j=0; j<i; j++ )
  {
    int v=buf[j];
    Serial.print(v/16, HEX);
    Serial.print(v%16, HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
#endif

  if ( buf[0] != 0xAA )
  {
    s = SatConnect::errorNoSpot;
    return s;
  }

  //Serial.print( "buf[7]=" ); Serial.println( buf[7] );

  if ( buf[7] == 7 )
  {
    s = SatConnect::findingPostion;
  }

  if ( buf[7] == 15 )
  {
    s = SatConnect::sending;
  }

  if ( buf[7] == 6 )
  {
    s = SatConnect::sending;
    if ( buf[19] == 1 )
    {
      s = SatConnect::sentOnce;
    }
    if ( buf[19] == 2 )
    {
      s = SatConnect::sentTwice;
    }
  }

  // TODO - detect error no GPS 
  if ( buf[7] == 0 )
  {
    s = SatConnect::readyToSend;
  }

  return s;
}


void SatConnect::begin()
{
  s = SatConnect::off;
  //serial.begin( 115200 ); // TODO - move to here ?

  Serial.println( "Starting up spot" );
  pinMode(pwrPin,OUTPUT);
  pinMode(onPin,OUTPUT);
  digitalWrite( pwrPin, LOW ); // turn off power to SPOT
  digitalWrite( onPin, HIGH ); // release the ON button for SPOT  
  delay( 1500 /* don't know how long this should be */ ); // wait for total power off for reset (500 seemed too low)
  //Serial.println( "  about to turn on power to spot" );
  digitalWrite( pwrPin, HIGH ); // turn on power to SPOT
  //Serial.println( "  power is on" );
  delay( 750 /* don't know how long this should be */ ); // wait for power to get stable 
  //Serial.println( "  turing on " );
  digitalWrite( onPin, LOW ); // press the ON button to power up SPOT
  delay( 4100 /* TODO - how long */ ); // wait for turn on and go into discovery mode TOOD - check if really need to go bluetooth discovery mode 
  //Serial.println( "  waiting to go on " );
  digitalWrite( onPin, HIGH ); // release the ON button for SPOT - it is still on 
  Serial.println( "   spot is on" );

  //unsigned long  startTime = millis();
  //unsigned long  charTime = millis();

  Serial.println( "Wait for SPOT self check and flush serial data" );
  unsigned long lastTime = millis();
  do
  {
    uint8_t c;
    if ( serial.available() )
    {
      c = serial.read();
      //charTime = millis();

      int v=c;
      Serial.print(v/16, HEX);
      Serial.print(v%16, HEX);
      Serial.print(" ");
    }
  } 
  while ( millis() < lastTime+3000 ); // flush for at least 2.5 seconds 
  Serial.println( "  done flushing" );

  //Serial.print( "flush time was " ); Serial.println( charTime - startTime );

  s = SatConnect::poweringUp;   
}


void SatConnect::end()
{
  if ( s ==  SatConnect::off );
  {
    digitalWrite( pwrPin, LOW ); // turn off the power to SPOT
    delay( 1000 ); // make power stays off of a bit 
    return;  
  }

  Serial.println( "Shuting down spot" );
  digitalWrite( onPin, LOW ); // press the ON button to power off SPOT
  delay( 2200  );   // 1500 too short 
  Serial.println( "  waiting to go off " );
  digitalWrite( onPin, HIGH ); // release the ON button for SPOT - should be off now  
  delay( 1500 /* TODO - how long */ );   
  digitalWrite( pwrPin, LOW ); // turn off the power to SPOT
  delay( 1000 ); // make power stays off of a bit 

  s = SatConnect::off;
  Serial.println( "   spot is off" );
}


void  SatConnect::write( char* msg, int len )
{
  if ( state() == SatConnect::off ) 
  {
    begin(); // if not powered on, turn it on 
  }

  do 
  { 
    // wait until ready to send
  } 
  while ( !ready() );

  if ( len > 41 ) // TODO - figure out max len 
  {
    len = 41; 
  }

  // send message to SPOT 
  serial.flush();
  static uint8_t sendReq[8] = { 
    0xAA, 0x08, 0x26, 0x01, 0x00, 0x01, 0x00, 0x01                               };
  sendReq[1] = sizeof(sendReq) + len; // set the length of the message 
  serial.write( sendReq, sizeof( sendReq ) );
  serial.write( (uint8_t*)msg, len );

  unsigned long lastTime = millis();
  uint8_t buf[ 0x30 ];
  buf[0] = 0;
  buf[1] = sizeof(buf);
  int i=0;
  do
  {
    if ( serial.available() )
    {
      buf[i++] = serial.read();
    }
    if ( i == buf[1] )
    {
      //Serial.println( "Read full packet" );
      break;
    }
  } 
  while ( (millis() < lastTime + 500) && ( i < sizeof(buf) ) ); // 500 ms timeout after reqest sent 

  if ( i != buf[1] )
  {
    int timeout = millis()-lastTime;
    Serial.println( "Problem reading packet buf after write" );
    Serial.print( "buf[1]=" ); 
    Serial.println( buf[1] );
    Serial.print( "i=" ); 
    Serial.println( i );
    Serial.print( "time out=" ); 
    Serial.println( timeout );
  }

#if 1
  //Serial.print( "i= 0x" ); 
  //Serial.println( i , HEX );
  Serial.print( "SPOT data after write: " );
  for ( int j=0; j<i; j++ )
  {
    int v=buf[j];
    Serial.print(v/16, HEX);
    Serial.print(v%16, HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
#endif

  s = SatConnect::haveMsgToSend;
}


SatConnect satConnect( Serial3, spotPowerPin, spotOnPin ); 

// =================================================================
// stuff for wind 

volatile unsigned long windCount; // count of revolutions of wind sensor 
unsigned long windStartTime;
unsigned long windPrevTime;
unsigned long windStartCount;
unsigned long windPrevCount;
unsigned int  minMetersPer10s=0;
unsigned int  curMetersPer10s=0;
unsigned int  maxMetersPer10s=0;
unsigned int  avgMetersPer10s=0;


void incWindCount()
{
  // this is the itnerupt handler and interupts are disabled in it
  windCount++; 
}


unsigned long getWindCount()
{
  unsigned long r;
  noInterrupts(); // disable interupts while copying the counter 
  r = windCount;
  interrupts();
  return r; 
}


void resetWind()
{
  unsigned long time = millis();
  unsigned long count = getWindCount();

  windStartTime = time;
  windPrevTime = time;
  windStartCount = count;
  windPrevCount = count; 

  minMetersPer10s = 0xFFFF;
  maxMetersPer10s = 0;
}


void updateWind()
{
  unsigned long time = millis();

  if ( time < windPrevTime + 10000 ) // wait at least 10 seconds
  {
    return;
  }
  unsigned long count = getWindCount();

  unsigned long dTime = time - windPrevTime;
  unsigned long dCount = count - windPrevCount;
  unsigned long dMeters = (int)dCount + ( (int)dCount / 8 ); // sketchy conversion that looks about right 
  curMetersPer10s = dMeters * 10000 / (int)dTime ;  // 10000 is 10 secons and 1000 to move time from ms to s

  if ( curMetersPer10s > maxMetersPer10s ) maxMetersPer10s = curMetersPer10s;
  if ( curMetersPer10s < minMetersPer10s ) minMetersPer10s = curMetersPer10s;

  //Serial.print( "dTime = " ); 
  //Serial.println( dTime );

  //Serial.print( "dCount = " ); 
  //Serial.println( dCount );

  //Serial.print( "Current = " ); 
  //Serial.println( curMetersPer10s );

  dTime = (time - windStartTime) / 100; // in 10th of seconds 
  dCount = count - windStartCount;
  unsigned long avgCountPer10s =  dCount*100 / dTime; // 100 is 10 for per 10 seconds , and 10 for time is in 10ths of seconds
  avgMetersPer10s = (int)avgCountPer10s + ( (int)avgCountPer10s / 8 ); // sketchy conversion that looks about right 

  //Serial.print( "Avg = " ); 
  //Serial.println( avgMetersPer10s );

  windPrevTime = time;
  windPrevCount = count;
}


//===============================================================================
// Stuff for battery 

unsigned int voltageX100=0;

void updateBattery()
{
  int v = analogRead( batteryPin ); // 0-5 v scaled to 0-1023 

  voltageX100 = 80 + v + (v>>1) + (v>>5); 
}


// ===============================================================================
// Stuff for temperature   

int          tempatureX10=0; 
typedef uint8_t DeviceAddr[8];
DeviceAddr tempatureAddr;
OneWire  oneWireBus(10); // using digiital IO on pin 10


void copyAddr( uint8_t src[], uint8_t dst[] )
{
  for( int i=0; i<8; i++ )
  {
    dst[i]=src[i];
  }
}


void setupTemp()
{
  tempatureX10 = 0;

  DEBUG("Scanning 1-wire Bus A ...");
  DeviceAddr addr;
  oneWireBus.reset_search();
  while ( oneWireBus.search(addr) )
  {
    if ( OneWire::crc8( addr, 7) != addr[7]) 
    {
      Serial.println("BAD-CRC2");
    }
    else
    {
      if ( addr[0] == 0x10) 
      {
        DEBUG("Found DS18S20 Temperature on bus A");
        copyAddr( addr, tempatureAddr);       
      }
      if ( addr[0] == 0x28) 
      {
        DEBUG("Found DS18B20 Temperature on bus A");
        copyAddr( addr, tempatureAddr);       
      }
      else
      {
        DEBUG2("Found one wire device of type address: ",  addr[0] );
      }
    }
  }
}


void updateTemp( )
{
  oneWireBus.reset();
  oneWireBus.select(tempatureAddr);
  oneWireBus.write(0x44,1);  // do conversion 

  delay(750);  // 750 ms for conversion time   

  oneWireBus.reset();
  oneWireBus.select(tempatureAddr);    
  oneWireBus.write(0xBE);  // read results

  byte data[9];
  for ( int i = 0; i < 9; i++) 
  {          
    data[i] = oneWireBus.read();
  }
  if ( OneWire::crc8( data, 8) != data[8] )
  {
    Serial.println("BAD-CRC");
    tempatureX10 = 0;
    return ; 
  }

  int temp16x = (data[1]<<8) + data[0];
  tempatureX10 = (temp16x*10) / 16;

  //DEBUG2( "Got tempature 10x = " , tempatureX10 );
}


// ========================================================================
// stuff to form the output JSON sent over SPOT 

char  outputVector[41];
void formatOutputVector()
{
  int i=0;
  outputVector[i++] = '{';
  outputVector[i++] = '"';
  outputVector[i++] = 'v';
  outputVector[i++] = '"';
  outputVector[i++] = ':';
  outputVector[i++] = '[';

  outputVector[i++] = '0' + (curMetersPer10s/100)%10;
  if  (outputVector[i-1] == '0' ) outputVector[i-1]=' ';
  outputVector[i++] = '0' + (curMetersPer10s/10 )%10;
  outputVector[i++] = '.';
  outputVector[i++] = '0' + (curMetersPer10s    )%10;
  outputVector[i++] = ',';

  outputVector[i++] = '0' + (minMetersPer10s/100)%10;
  if  (outputVector[i-1] == '0' ) outputVector[i-1]=' ';
  outputVector[i++] = '0' + (minMetersPer10s/10 )%10;
  outputVector[i++] = '.';
  outputVector[i++] = '0' + (minMetersPer10s    )%10;
  outputVector[i++] = ',';

  outputVector[i++] = '0' + (avgMetersPer10s/100)%10;
  if  (outputVector[i-1] == '0' ) outputVector[i-1]=' ';
  outputVector[i++] = '0' + (avgMetersPer10s/10 )%10;
  outputVector[i++] = '.';
  outputVector[i++] = '0' + (avgMetersPer10s    )%10;
  outputVector[i++] = ',';

  outputVector[i++] = '0' + (maxMetersPer10s/100)%10;
  if  (outputVector[i-1] == '0' ) outputVector[i-1]=' ';
  outputVector[i++] = '0' + (maxMetersPer10s/10 )%10;
  outputVector[i++] = '.';
  outputVector[i++] = '0' + (maxMetersPer10s    )%10;
  outputVector[i++] = ',';

  // output tempature in degC  
  int v = (tempatureX10>=0) ? tempatureX10 : -tempatureX10;
  outputVector[i++] = (tempatureX10>0) ? ' ' : '-';
  outputVector[i++] = '0' + (v/100)%10;
  if  (outputVector[i-1] == '0' ) 
  {
    outputVector[i-1]= (tempatureX10>=0) ? ' ' : '-';
    outputVector[i-2]= ' ';
  }
  outputVector[i++] = '0' + (v/10 )%10;
  outputVector[i++] = '.';
  outputVector[i++] = '0' + (v    )%10;
  outputVector[i++] = ',';

  // output battery voltage voltageX100
  outputVector[i++] = '0' + (voltageX100/1000)%10;
  if  (outputVector[i-1] == '0' ) outputVector[i-1]=' ';
  outputVector[i++] = '0' + (voltageX100/100  )%10;
  outputVector[i++] = '.';
  outputVector[i++] = '0' + (voltageX100/10   )%10;
  outputVector[i++] = '0' + (voltageX100      )%10;
  //outputVector[i++] = ',';

  outputVector[i++] = ']';
  outputVector[i++] = '}';
  outputVector[i++] = 0;
  if (i >= sizeof(outputVector) )
  {
    Serial.println("buffer overlow in format output vector");
    DEBUG2( "i=" , i );
    while (1);
  }
}



//=====================================================================
// main program part 

void setup()
{
  Serial.begin(19200); 
  Serial3.begin(115200); // important - don't forget to setup the serial connection speed to the spot 

  delay( 500 ); 
  Serial.println("Starting program");

  // setup wind 
  pinMode(windPin, INPUT);
  windCount = 0;
  attachInterrupt(0, incWindCount, RISING); // int 0 is digital pin 2 
  resetWind();

  // set up battery 
  analogReference( DEFAULT );

  // set up temperature on oneWire 
  setupTemp();
}


void loop()
{
  static bool sendNow = true;
  static unsigned long prevLoopTime;
  unsigned long thisLoopTime;

  unsigned long thisHour = prevLoopTime / 3600000;
  unsigned long prevHour = thisLoopTime / 3600000;

  // every hour, send a new report
  if ( thisHour != prevHour )
  {
    sendNow = true; 
  }

  // if the spot has and error, shut it down 
  if ( satConnect.state() == SatConnect::errorNoSpot )
  {
    DEBUG( "turned off sat due to error" );
    satConnect.end(); // turn power to spot off 
    sendNow = false; // if msg to be sent, wait till next hour
  }

  // if message to send,
  if (sendNow)
  {
    if (  millis() > 30000  )  // if message to send, and arduino has been up for at least 30 seconds, turn on the spot
    {
      if ( satConnect.state() == SatConnect::off  )
      {
        DEBUG( "turned on sat because have messages to send" );
        satConnect.begin(); // turn spot power on
      }
    }
  }

  if (sendNow)
  {
    if ( satConnect.ready()  )
    {
      updateBattery();
      updateTemp();
      updateWind();

      formatOutputVector();

      resetWind();

      Serial.print("Sending Message len="); 
      Serial.print( strlen(outputVector) ); 
      Serial.print(" "); 
      Serial.println( outputVector );

      satConnect.write( outputVector, strlen(outputVector) );
      sendNow = false;
    }
  }

#if 1 // TODO remove 
  if ( sendNow == false )
  { 
    if ( satConnect.state() == SatConnect::sentOnce)
    {
      DEBUG( "Turning off sat because have sent one message" );
      satConnect.end(); // turn power to spot off 
    }
  }
#endif  
  
  if ( sendNow == false )
  { 
    if ( satConnect.ready() )
    {
      DEBUG( "Turning off sat because done sending messages" );
      satConnect.end(); // turn power to spot off 
    }
  }

#if 1
  int state = int( satConnect.state() );
  unsigned long time =  millis();

  Serial.print( "Time " );
  Serial.print( time );
  Serial.print( ": " );
  switch ( state )
  {
    case SatConnect::off:      
    {  
      Serial.println( "Spot connect is off" ); 
      break;
    }
    case SatConnect::poweringUp:      
    {  
      Serial.println( "Spot connect is poweringUp" ); 
      break;
    }
    case SatConnect::readyToSend:      
    {  
      Serial.println( "Spot connect is readyToSend" ); 
      break;
    }
    case SatConnect::haveMsgToSend:      
    {  
      Serial.println( "Spot has msg ready to send" ); 
      break;
    }
    case SatConnect::findingPostion:      
    {  
      Serial.println( "Spot connect is findingPostion" ); 
      break;
    }
    case SatConnect::sending:      
    {  
      Serial.println( "Spot connect is sending" ); 
      break;
    }
    case SatConnect::sentOnce:      
    {  
      Serial.println( "Spot connect is sentOnce" ); 
      break;
    }
    case SatConnect::sentTwice:      
    {  
      Serial.println( "Spot connect is sentTwice" ); 
      break;
    }
    case SatConnect::errorNoGPS:      
    {  
      Serial.println( "Spot connect has error No GPS signal" ); 
      break;
    }
    case SatConnect::errorNoSpot:      
    {  
      Serial.println( "Spot connect has error No Spot device" ); 
      break;
    }
  }
#endif

  updateWind();

#if 1 // print out current conditions ever 5 minutes 
  unsigned long thisFiveMin = prevLoopTime / 300000;
  unsigned long prevFiveMin = thisLoopTime / 300000;
  if ( thisFiveMin != prevFiveMin )
  {
    updateBattery();
    updateTemp(); // calling this too often warms up the sensor resulting in bogus measurements

    formatOutputVector();

    Serial.print("Time = "); 
    Serial.print( millis()/1000 );
    Serial.print("  Status = "); 
    Serial.println( outputVector );
  }
#endif

  delay( 1500 ); 

  prevLoopTime = thisLoopTime;
}































