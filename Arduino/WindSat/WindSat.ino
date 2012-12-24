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

#define DEBUG 

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
  unsigned long lastTime;
};


SatConnect::SatConnect( Stream& pSerial, int pPwrPin, int pOnPin ): 
serial(pSerial), pwrPin(pPwrPin), onPin(pOnPin)
{
  pinMode(pwrPin,OUTPUT);
  digitalWrite( pwrPin, LOW ); // turn off power to SPOT
  s = SatConnect::off;
  lastTime = millis();
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
  //Serial.print( "    In state() lastTime = " ); 
  //Serial.println( lastTime );

  if  (  s == SatConnect::off )
  {
    return s;
  }

  if ( millis() < lastTime + 500 ) // if state if less than 500 ms old, don't check for new state 
  { 
    return s;
  }

  // flush out any old stuff in serial buffer   
  lastTime = millis();
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
    0xAA, 0x03, 0x52     };
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
  while ( (millis() < lastTime + 150) && ( i < sizeof(buf) ) ); // 100 ms timeout 

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

#ifdef DEBUG
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

  lastTime = millis();
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
  delay( 500 /* don't know how long this should be */ ); // wait for total power off for reset
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
  lastTime = millis();
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
  lastTime = millis();
}


void SatConnect::end()
{
  Serial.println( "Shuting down spot" );
  digitalWrite( onPin, LOW ); // press the ON button to power off SPOT
  delay( 2200  );   // 1500 too short 
  Serial.println( "  waiting to go off " );
  digitalWrite( onPin, HIGH ); // release the ON button for SPOT - should be off now  
  delay( 1500 /* TODO - how long */ );   
  digitalWrite( pwrPin, LOW ); // turn off the power to SPOT
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
    0xAA, 0x08, 0x26, 0x01, 0x00, 0x01, 0x00, 0x01     };
  sendReq[1] = sizeof(sendReq) + len; // set the length of the message 
  serial.write( sendReq, sizeof( sendReq ) );
  serial.write( (uint8_t*)msg, len );

  lastTime = millis();
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
  while ( (millis() < lastTime + 150) && ( i < sizeof(buf) ) ); // 150 ms timeout 

  if ( i != buf[1] )
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

#ifdef DEBUG
  //Serial.print( "i= 0x" ); 
  //Serial.println( i , HEX );
  Serial.print( "SPOT Send: " );
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


SatConnect satConnect( Serial3, 6 /* PwrPin */ , 7 /* OnPin */ ); 


void setup()
{
  Serial.begin(19200); 
  Serial3.begin(115200); // important - don't forget to setup the serial connection speed to the spot 

  delay( 500 ); 

  Serial.println("Starting program");
  satConnect.begin(); // turn spot power on
}


void loop()
{
  static bool sent = false;

  if (!sent)
  {
    if ( satConnect.ready()  )
    {

      char* msg= "{\"e\":[{\"n\":\"avgWind\",\"v\":6.104}]}"; // senml json format 

      Serial.print("Sending Message len="); 
      Serial.print( strlen(msg) ); 
      Serial.print(" "); 
      Serial.println( msg );

      satConnect.write( msg, strlen(msg) );
      sent = true;

    }
  }
  else
  {
    if ( satConnect.ready() )
    {
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
      Serial.println( "Spot has msg q'd to send" ); 
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
      Serial.println( "Spot connect is errorNoGPS" ); 
      break;
    }
    case SatConnect::errorNoSpot:      
    {  
      Serial.println( "Spot connect is errorNoSpot" ); 
      break;
    }
  }
#endif

  delay( 1500 ); 
}


















