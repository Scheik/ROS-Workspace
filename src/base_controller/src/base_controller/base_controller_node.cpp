#include <iostream>
#include <stdio.h>   								/* Standard input/output definitions */
#include <stdlib.h>  								/* exit */
#include <string.h>  								/* String function definitions */
#include <unistd.h>  								/* UNIX standard function definitions */
#include <fcntl.h>  	 							/* File control definitions */
#include <errno.h>   								/* Error number definitions */
#include <termios.h> 								/* POSIX terminal control definitions */
#include <ctype.h>   								/* isxxx() */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>


struct termios orig;
int filedesc;
int fd;
unsigned char serialBuffer[16];						// Serial buffer to store data for I/O

int openSerialPort(const char * device, int bps)
{
   struct termios neu;
   char buf[128];

   fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

   if (fd == -1)
   {
      sprintf(buf, "openSerialPort %s error", device);
      perror(buf);
   }
   else
   {
      tcgetattr(fd, &orig); 						/* save current serial settings */
      tcgetattr(fd, &neu);
      cfmakeraw(&neu);
      //fprintf(stderr, "speed=%d\n", bps);
      cfsetispeed(&neu, bps);
      cfsetospeed(&neu, bps);
      tcsetattr(fd, TCSANOW, &neu); 				/* set new serial settings */
      fcntl (fd, F_SETFL, O_RDWR);
   }
   return fd;
}

void writeBytes(int descriptor, int count) {
    if ((write(descriptor, serialBuffer, count)) == -1) {	// Send data out
        perror("Error writing");
        close(descriptor);				// Close port if there is an error
        exit(1);
    }
    //write(fd,serialBuffer, count);

}

void readBytes(int descriptor, int count) {
    if (read(descriptor, serialBuffer, count) == -1) {	// Read back data into buf[]
        perror("Error reading ");
        close(descriptor);				// Close port if there is an error
        exit(1);
    }
}


void read_MD49_Data (void){
    serialBuffer[0] = 82;							// 82=R Steuerbyte um alle Daten vom MD49 zu lesen
    writeBytes(fd, 1);
    //usleep(400000);
    //Daten lesen und in Array schreiben
    readBytes(fd, 18);
    printf("\033[2J");        /*  clear the screen  */
    printf("\033[H");         /*  position cursor at top-left corner */
    printf ("MD49-Data read from AVR-Master: \n");
    printf("====================================================== \n");
    printf("Encoder1 Byte1: %i ",serialBuffer[0]);
    printf("Byte2: %i ",serialBuffer[1]);
    printf("Byte3: % i ",serialBuffer[2]);
    printf("Byte4: %i \n",serialBuffer[3]);
    printf("Encoder2 Byte1: %i ",serialBuffer[4]);
    printf("Byte2: %i ",serialBuffer[5]);
    printf("Byte3: %i ",serialBuffer[6]);
    printf("Byte4: %i \n",serialBuffer[7]);
    printf("====================================================== \n");
    printf("Speed1: %i ",serialBuffer[8]);
    printf("Speed2: %i \n",serialBuffer[9]);
    printf("Volts: %i \n",serialBuffer[10]);
    printf("Current1: %i ",serialBuffer[11]);
    printf("Current2: %i \n",serialBuffer[12]);
    printf("Error: %i \n",serialBuffer[13]);
    printf("Acceleration: %i \n",serialBuffer[14]);
    printf("Mode: %i \n",serialBuffer[15]);
    printf("Regulator: %i \n",serialBuffer[16]);
    printf("Timeout: %i \n",serialBuffer[17]);
}


void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{ 
    ROS_INFO("I heard: [%f]", vel_cmd.linear.y);
    std::cout << "Twist Received " << std::endl;

    //hier code um msg in seriellen Befehl umzuwandeln

        //code

    //
}


int main( int argc, char* argv[] )
{
ros::init(argc, argv, "base_controller" );

ros::NodeHandle n;
ros::Subscriber sub = n.subscribe("/cmd_vel", 1000, cmd_vel_callback);

filedesc = openSerialPort("/dev/ttyAMA0", B38400);
if (filedesc == -1) exit(1);
usleep(40000);									// Sleep for UART to power up and set options

ROS_INFO_STREAM("serial Port opened \n");


while( n.ok() ) 
{
    read_MD49_Data();
    usleep(100000);
    ros::spin();
    //hier code/funktion um md49 daten zu lesen und und auszugeben in konsole und /odom später

}

return 1;
}
