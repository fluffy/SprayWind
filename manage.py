#!/usr/bin/env python2.7

import os
import sys

sys.path.append('/home/fluffy/webapps/spraywind')
sys.path.append('/home/fluffy/webapps/spraywind/SprayWind')
sys.path.append('/home/fluffy/webapps/spraywind/lib/python2.7')

sys.path.append('/Users/fluffy/src/SprayWind')
sys.path.append('/usr/local/bin/python2.7')


if __name__ == "__main__":
    os.environ.setdefault("DJANGO_SETTINGS_MODULE", "website.settings")

    from django.core.management import execute_from_command_line

    execute_from_command_line(sys.argv)
