'''
 Copyright (c) 2012, Cullen Jennings <fluffy@iii.ca> All rights reserved.
 
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
'''

from django.http import HttpResponse
from django.shortcuts import render
import datetime
import json
import string

from website.wind.models import SensorReading


def getWind( request, sensorName ):
    sensorReading = SensorReading.objects.filter( sensorID=sensorName ).latest()
    return render(request, 'wind.html', 
    				{ 'time':    sensorReading.time, 
    				  'curWind': sensorReading.curWind*1.943, # convert from m/s to knot
    				  'minWind': sensorReading.minWind*1.943,
    				  'avgWind': sensorReading.avgWind*1.943,
    		          'maxWind': sensorReading.maxWind*1.943,
    				  'temperature':sensorReading.temperature ,
    				  'voltage': sensorReading.voltage  } )


def getInfo( request, sensorName ):
    sensorReadings = SensorReading.objects.filter( sensorID=sensorName ).order_by( '-time' ).all()[:24]
    html = "<html><body><pre>\n" 
    for sensorReading in sensorReadings:
	    html += sensorReading.info
    html += "\n</pre></body></html>"
    return HttpResponse( html )



def rockBlock( request ):
	sensorName = "sprayWind"
  	now = datetime.datetime.now()

	log =  "rockBlock at %s \n" % now
	log += "request method = %s \n" % request.method 
	log += "request body = %s \n\n" % request.body 

    log += "imei = %s \n" % request.POST.get("imei");
    log += "momsn = %s \n" % request.POST.get("momsn");
    log += "transmit_time = %s \n" % request.POST.get("transmit_time");
    log += "iridium_latitude = %s \n" % request.POST.get("iridium_latitude");
    log += "iridium_longitude = %s \n" % request.POST.get("iridium_longitude");
    log += "iridium_cep = %s \n" % request.POST.get("iridium_cep");
    log += "data = %s \n" % request.POST.get("data");
     
	curWind = -1.0
	minWind = -1.0
	avgWind = -1.0
	maxWind = -1.0
	curTemp = 0.0
	curVoltage = 0.0


    log += "wind %s (min,avg,max) = %s,%s,%s temp = %s battery = %s \n"%( curWind, minWind, avgWind, maxWind , curTemp, curVoltage )
 	r = SensorReading( sensorID=sensorName, time=now, info=log, 
 			curWind=curWind, minWind=minWind, avgWind=avgWind, maxWind=maxWind, temperature=curTemp, voltage=curVoltage )
  	r.save()

	return HttpResponse( )




def cloudMailInJson( request ):
	sensorName = "sprayWind"
  	now = datetime.datetime.now()

	log =  "cloudMainInJson at %s \n" % now
	log += "request method = %s \n" % request.method 
	log += "request body = %s \n\n" % request.body 

	curWind = -1.0
	minWind = -1.0
	avgWind = -1.0
	maxWind = -1.0
	curTemp = 0.0
	curVoltage = 0.0 

	try:
		email = json.loads( request.body )
		body = email[ 'plain' ]
		senml = body[ string.find(body,"{") : string.rfind(body,"}")+1 ]
		sensor = json.loads( senml )

		#log += "senml = %s \n" % json.dumps( sensor ) 

		#for reading in sensor[ 'e' ]:
		#	if reading['n'] == "minWind":  minWind = reading['v']
		#	if reading['n'] == "avgWind":  avgWind = reading['v']
		#	if reading['n'] == "maxWind":  maxWind = reading['v']

		log += "json = %s \n" % json.dumps( sensor ) 

		v = sensor[ 'v' ]
		( curWind, minWind, avgWind, maxWind, curTemp, curVoltage ) = v
	
	except Exception as e:
		log += "Problem parsing the email. Exception %s \n"%e

	log += "wind %s (min,avg,max) = %s,%s,%s temp = %s battery = %s \n"%( curWind, minWind, avgWind, maxWind , curTemp, curVoltage )
 	r = SensorReading( sensorID=sensorName, time=now, info=log, 
 			curWind=curWind, minWind=minWind, avgWind=avgWind, maxWind=maxWind, temperature=curTemp, voltage=curVoltage )
  	r.save()

	return HttpResponse( )
