from django.http import HttpResponse
from django.shortcuts import render
import datetime

from website.wind.models import SensorReading

def getWind( request, sensorName ):

    #now = datetime.datetime.now()
    #r = SensorReading( sensorID=sensorName, time=now, minWind=1.0, avgWind=2.0, maxWind=3.0 )
    #r.save()

    sensorReading = SensorReading.objects.get( sensorID=sensorName )
    time = sensorReading.time 
    avgWind = sensorReading.avgWind

    return render(request, 'wind.html', 
    				{ 'time':time, 'avgWind':avgWind } )