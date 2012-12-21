from django.db import models

class SensorReading( models.Model ):
	sensorID = models.CharField( max_length=64, primary_key=True, unique=True )
	version  = models.IntegerField( default=1 )
	time     = models.DateTimeField( auto_now=False )
	maxWind  = models.FloatField()
	avgWind  = models.FloatField()
	minWind  = models.FloatField()
