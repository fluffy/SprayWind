
git pull

chown -R www-data /root/SprayWind

cp /root/SprayWind/sprayWind-uwsgi.ini /etc/uwsgi/apps-available/sprayWind-uwsgi.ini

cp /root/SprayWind/sprayWind-nginx.conf /etc/nginx/sites-available/sprayWind-nginx.conf

systemctl restart  uwsgi

systemctl restart  nginx
