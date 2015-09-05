import os
import sys

sys.path.append('/root/SprayWind')
sys.path.append('/usr/lib/python2.7/dist-packages')

os.environ['DJANGO_SETTINGS_MODULE'] = 'website.settings'

from django.core.wsgi import get_wsgi_application
application = get_wsgi_application()

