from django.http import HttpResponse
from django.shortcuts import render
import datetime
import json
import string

from website.wind.models import SensorReading


def getWind( request, sensorName ):
    sensorReading = SensorReading.objects.get( sensorID=sensorName )
    return render(request, 'wind.html', 
    				{ 'time':sensorReading.time, 
    				  'avgWind':sensorReading.avgWind } )


def getInfo( request, sensorName ):
    sensorReading = SensorReading.objects.get( sensorID=sensorName )
    return HttpResponse( "<html><body><pre>\n" + sensorReading.info + "\n</pre></body></html>" )


def cloudMailInJson( request ):
	sensorName = "sprayWind"
  	now = datetime.datetime.now()

	log =  "cloudMainInJson at %s \n" % now
	log += "request method = %s\n" % request.method 
	log += "request body = %s \n\n" % request.body 

	minWind = -1.0
	avgWind = -1.0
	maxWind = -1.0

	try:
		email = json.loads( request.body )
		body = email[ 'plain' ]
		senml = body[ string.find(body,"{") : string.rfind(body,"}")+1 ]
		sensor = json.loads( senml )

		log += "senml = %s \n" % json.dumps( sensor ) 

		for reading in sensor[ 'e' ]:
			if reading['n'] == "minWind":  minWind = reading['v']
			if reading['n'] == "avgWind":  avgWind = reading['v']
			if reading['n'] == "maxWind":  maxWind = reading['v']
	except Exception as e:
		log += "Problem parsing the email. Exception %s \n"%e

	log += "wind (min,avg,max) = %s,%s,%s \n"%( minWind, avgWind, maxWind)
 	r = SensorReading( sensorID=sensorName, time=now, info=log, minWind=minWind, avgWind=avgWind, maxWind=maxWind )
  	r.save()

	return HttpResponse( )
