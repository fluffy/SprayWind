from django.http import HttpResponse
from django.shortcuts import render
import datetime

from website.wind.models import SensorReading


def getWind( request, sensorName ):

    sensorReading = SensorReading.objects.get( sensorID=sensorName )

    return render(request, 'wind.html', 
    				{ 'time':sensorReading.time, 
    				  'avgWind':sensorReading.avgWind } )


def getInfo( request, sensorName ):

    sensorReading = SensorReading.objects.get( sensorID=sensorName )

    return HttpResponse( "<html><body>\n" + sensorReading.info + "\n</body></html>" )


def cloudMailInJson( request ):
	sensorName = "sprayWind"
  	now = datetime.datetime.now()

	log =  "cloudMainInJson at %s \n" % now
	log += "request method = " + request.method + "\n"
	log += "request body = " + request.body + "\n"

 	r = SensorReading( sensorID=sensorName, time=now, info=log, minWind=1.0, avgWind=2.0, maxWind=3.0 )
  	r.save()

	return HttpResponse( )
