
#echo install ssh authorized keys to login 

#sudo echo "fluffy ALL = NOPASSWD: ALL"  > /etc/sudoers.d/cullen ; 
#sudo chmod 0440 /etc/sudoers.d/cullen 
#sudo echo "fluffy ALL=(postgres) NOPASSWD: ALL"  >> /etc/sudoers.d/cullen

sudo apt-get update
   
sudo apt-get -y install ufw
sudo ufw default deny
sudo ufw logging on
sudo ufw allow ssh/tcp
sudo ufw limit ssh/tcp
sudo ufw allow http/tcp
sudo ufw allow https/tcp
#sudo ufw allow http-alt/tcp
echo y | sudo ufw enable

sudo apt-get -y upgrade
#echo you might want to reboot your server about here 


sudo apt-get -y install fail2ban
sudo apt-get -y install logcheck logcheck-database

echo "America/Edmonton" | sudo tee /etc/timezone
sudo dpkg-reconfigure --frontend noninteractive tzdata

#sudo apt-get -y remove ax25-node libax25

sudo apt-get -y install emacs24-nox git-core build-essential gcc tcsh

ssh-keyscan -H github.com >> ~/.ssh/known_hosts

sudo apt-get -y install curl 

#sudo sh -c 'echo "deb http://stable.packages.cloudmonitoring.rackspace.com/ubuntu-14.04-x86_64 cloudmonitoring main" > /etc/apt/sources.list.d/rackspace-monitoring-agent.list'
#curl https://monitoring.api.rackspacecloud.com/pki/agent/linux.asc | sudo apt-key add -
#sudo apt-get update
#sudo apt-get install rackspace-monitoring-agent

#sudo apt-get -y install mongodb
#sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 7F0CEB10
#sudo sh -c 'echo "deb http://downloads-distro.mongodb.org/repo/ubuntu-upstart dist 10gen" > /etc/apt/sources.list.d/mongodb.list '
#sudo apt-get update
#sudo apt-get -y install mongodb-org

#sudo apt-get -y install redis
#sudo apt-get -y install redis-server
#sudo cp /etc/redis/redis.conf /etc/redis/redis.conf.default
#sudo service redis-server restart

#curl http://www.rabbitmq.com/rabbitmq-signing-key-public.asc  | sudo apt-key add -
#sudo sh -c 'echo "deb http://www.rabbitmq.com/debian/ testing main" > /etc/apt/sources.list.d/rabbitmq.list '
#sudo apt-get update
#sudo apt-get -y install rabbitmq-server 

#sudo apt-get -y install nodejs
#sudo apt-get -y install npm 

# Forever seems to need this 
#sudo ln -s `which nodejs` /usr/local/bin/node
#sudo apt-get -y install node-legacy
#sudo npm install -g forever
#sudo npm install -g forever-monitor
    
#sudo npm install -g bower

# no idea why trying this twice is better
#sudo npm install -g forever
#sudo npm install -g forever-monitor

sudo apt-get -y install nginx 

sudo rm /etc/nginx/sites-enabled/default

sudo apt-get -y install python-django

cd ~
git clone https://github.com/fluffy/SprayWind.git

# copy setting.tmpl to settings.py

cp SprayWind/website/settings.tmpl SprayWind/website/settings.py
mkdir SprayWind/static


sudo apt-get -y install uwsgi 
sudo apt-get -y install uwsgi-plugin-python3
sudo apt-get -y install uwsgi-plugin-python

# adduser --system spraywind


cp /root/SprayWind/sprayWind-uwsgi.ini /etc/uwsgi/apps-available/sprayWind-uwsgi.ini
ln -s /etc/uwsgi/apps-available/sprayWind-uwsgi.ini /etc/uwsgi/apps-enabled/sprayWind-uwsgi.ini


cp /root/SprayWind/sprayWind-nginx.conf /etc/nginx/sites-available/sprayWind-nginx.conf
ln -s /etc/nginx/sites-available/sprayWind-nginx.conf /etc/nginx/sites-enabled/sprayWind-nginx.conf


#sudo service uwsgi restart
#sudo service nginx restart
#tail -f /var/log/nginx/access.log &
#tail -f /var/log/nginx/error.log &

# systemctl status  uwsgi

chmod a+rx /Root
chown -R www-data /root/SprayWind

# systemctl restart  uwsgi
# systemctl restart  nginx

# git pull ; chown -R www-data /root/SprayWind; cp /root/SprayWind/sprayWind-uwsgi.ini /etc/uwsgi/apps-available/sprayWind-uwsgi.ini ; cp /root/SprayWind/sprayWind-nginx.conf /etc/nginx/sites-available/sprayWind-nginx.conf


sudo  apt-get -y  install python-pip

pip install uwsgitop

# uwsgitop "127.0.0.1:7001"

