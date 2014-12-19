This folder keeps data used from package base_controller 
and is woorking as root directory for nginx (running on pi).

Therefore edit nginx defaults with: sudo nano /etc/nginx/sites-available/default

comment 

root /usr/share/nginx/www;

and add instead 

root /home/pi/ROS-Groovy-Workspace/data;

give rights for user www-data

sudo chown www-data.www-data ~/ROS-Groovy-Workspace/data

restart nginx

sudo /etc/init.d/nginx restart

Current content:

index.php : default website  if nginx is running and properly configured

phpliteadmin.php : makes databases accesable through browser if nginx is running and properly configured

md49data.db : SQLite3 database, keeping data read from MD49-Drive-Controller


