from django.conf.urls import patterns, include, url
from django.contrib import admin


admin.autodiscover()

urlpatterns = patterns('',
    url( r'^$', 'website.wind.views.getWind', { 'sensorName':"sprayWind" } ),
    url( r'^spot.json$', 'website.wind.views.cloudMailInJson' ),
 
    # url(r'^admin/doc/', include('django.contrib.admindocs.urls')),
    url( r'^admin/', include(admin.site.urls) ),
)
