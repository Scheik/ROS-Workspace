This folder keeps data used from package base_controller 
and is woorking as root directory for nginx.

Therefore edit nginx defaults with: sudo nano /etc/nginx/sites-available/default

comment sudo nano /etc/nginx/sites-available/default

and add instead 

root ~/ROS-Groovy-Workspace/data


Current content:

index.php : default website  if nginx is running and properly configured

phpliteadmin.php : makes databases accesable through browser if nginx is running and properly configured

md49data.db : SQLite3 database, keeping data read from MD49-Drive-Controller


