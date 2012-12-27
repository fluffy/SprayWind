
# Overview:

The systme uses an Arduino to sense the wind and tempature and then
connects to a SPOT Connect to send it up to a satalite. This results
in an email sent to cloudmailin which then does an HTTP POST to the
website with the email data. The website is a django app. The basic
block diagram of the modules is shown below.

![Block Diagram](block-diag.png "Block Diagram")

## Wind Sensor

The wind sensor has a reed switch that connects the circuit once per
revolution. 

## Interface Board

The inteface board conencts the rest of the stuff up to the
Arduino. It provide a 4.7 kOhm resistor
for the 1-wire bus to the tempature sensor. It has a 10k pull up to
provide power to the anemometer read switch. It connects serial port
TBD to SPOT board. It splits the battery voltage roughly in thrid and
monitors that.

Schematic is:


Photo is:


Jumper layout is:




## Power Supply

Currently using a "car battery". Actually it is battery that supports
"deep cycles" desinged more for boats and RVs than for starting cars.


Need to look into hwo much power is used at variso times. It looks
like roughly speaking, when SPOT is not in use, this consumes around
100 mA. This is very high and could be substantially reduced. The SPOT
looks like it consuems roudly another 100 mA on averager for 20
minutes each time a messages is sent. It is higher when the GPS is
trying to locate the position. It goes very high (perhaps 800 mA ?)
for a breif time when the messages is being trasmitted.

The voltage divider circuit does not have any over pwoer protection
(note, add zeneir diod on next design) so the input voltage must not
go over TBD volts. In adition the power supply on the Arduion Mega is
limited to 20 volts. 


## Arduino

The system is using an Arduino Mega 2560 cuase that was what was lying
around and it has muple serial ports but probably a normal Arduino would
work. If moving to a normal Arduino, I would put the debug informatio
on a softserial line and use the main UART to talk to the SPOT.

The reed switch on the anemometer generates and interupt which
icrements a counter. Aproximately every 10 seconds, this counter is
read to compute the current wind speed which is then used to update
the min and max wind. The counter is also used to compute an average
wind over a one hour period.

The battery voltage and current tempratuer are comptuted ever 5
minutes. [Note the current system is createing tempature measurements
that are way off].

Every hour, the system gets the current values, forms them into a JSON
message and sends it using the SPOT. If the SPOT does not manage to
send the messages within 30 minutes, it gets shutdwon as this is
probably some sort of weird error state on the SPOT. 

The JSON messages is a single JSON object that contains an array named
"v" that has 6 floating point values. The first is the current wind
speed in m/s, the second the minimuim over last hour, the next the
averge, then the maximuim. The next value is the tempature in degrees
C followed by the battery voltage in volts. An example JSON object is 

{"v":[ 0.0, 0.0, 0.0, 0.0,-10.4,12.38]}


## Data Log

This is optional but convenent for debuggin if things are not
working. All the debug informationm printed from the Arduino is
recorded on a flash card in the logger. Currenly using an OpenLog
<https://www.sparkfun.com/products/9530> to record all the debug
info. TODO - file size?

Right now the serial is configured for 19.2 kbps so the the CONFIG.TXT
files for the open log looks like: TODO


## Spot Interface

I'm using the SatUplink Shield from sparkfun
[https://www.sparkfun.com/products/11088]. This provides the varios
voltages the spot requires along with a way to turn them on and off. 

## Spot Connect

Before you take apart a spot, make sure it works with you phone, you
have the website and account set up and working and have upgraded to
latest firmware on the spot. 

One of the key problems with the SPOT system is that there is no
acknoledgment that the messages SPOT sent was received. This makes it
much harder to use and make it unclear how many times the messages
should be retrasmitted. It would be nice to reduce the number of
retransmitions as this consumes lots of power.


## Spot Website and Emai Gateway

The findmespot website is configured to send messages from my SPOT to
the email gateway at http://www.cloudmailin.com/ which is confiugred
to use the JSON format to post to the django website. [Note: some
trivial python code could just access an email account directly and
skip cloudmailin out of the picture. Plan to do that Real Soon Now.]

## Website

The website is written in python using django. I am using webfaction
to host it as I find the cost of doing it there way cheaper than GAE,
Heroku, Rackspace, or AWS. The website is using postgress for the
database but pretty much anything would work. 


## Connection on the box



# Cost of system

Very aproximate costs in US$ as of Dec 2012

Equipment
SPOT Device: 150
SatUplink Sheild: 21
Shipping: 25
Arduino Mega 2560: 60
Proto Sheild: 20
Tempature Sensor: 5  
Misc parts: 25
Battery: 100 
Things I forgot: 60
OpenLog: 26
FlashCard: 5
Total: about $500

Anual Fees
Spot Account: 100
Spot Messages (1/hour): 876
Domain Name: 15
DNS: 30 
Website: 120 
CloudMailIn: 108

Total: 1249
($3.42 / day - less than starbucks) 



# The next version:

Desing for low power. Don't use arduino power supplies. Include a real time
clock with alarm function to wake up the arduino so it can be put in
sleep mode. Do all 3 volt
design. Make sure sleep mode works. Remove all the LEDs from spot and 
arduino.

Think about input power protection battery sense circuit.

Think about local status reporting.

Real time clock to sync reporting on the hour and reduce reporting at
night. 

Figure out solar power.

Use magnets for mouting boards to box ?

Add local display of wind / voltage ?

Use soft serial out for debug information

Consider adding logging to SD card 

