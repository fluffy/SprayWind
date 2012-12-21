from django.conf.urls import patterns, include, url
from django.contrib import admin


admin.autodiscover()

urlpatterns = patterns('',
    # Examples:
    url( r'^$', 'website.wind.views.getWind', { 'sensorName':"sprayWind" } ),
    # url(r'^website/', include('website.foo.urls')),

    # url(r'^admin/doc/', include('django.contrib.admindocs.urls')),
    url( r'^admin/', include(admin.site.urls) ),
)
