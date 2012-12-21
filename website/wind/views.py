from django.http import HttpResponse
from django.shortcuts import render
import datetime

from website.wind.models import SensorReading

def getWind( request, sensorName ):

    now = datetime.datetime.now()
    r = SensorReading( sensorID=sensorName, time=now, minWind=1.0, avgWind=2.0, maxWind=3.0 )
    r.save()

    sensorReading = SensorReading.objects.get( sensorID=sensorName )

    return render(request, 'wind.html', 
    				{ 'time':sensorReading.time, 
    				  'avgWind':sensorReading.avgWind } )


def cloudMailInJson( request ):

	html = "<html><body> Mail IN is OK "
	html += "request method = " + request.method + "\n"
	html += "request body = " + request.body + "\n"
	html += "</body></html>" 

	return HttpResponse(html)
