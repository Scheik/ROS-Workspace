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

using namespace std;

void set_MD49_Commands (void);
void print_help(void);
void parse_input (void);

unsigned char set_Speed1=128;
unsigned char set_Speed2=128;
unsigned char set_Acceleration=5;
unsigned char set_Mode=0;
unsigned char set_Reset=0;
unsigned char set_Regulator=1;
unsigned char set_Timeout=0;
unsigned char set_Encoder1Byte1=0;
unsigned char set_Encoder1Byte2=0;
unsigned char set_Encoder1Byte3=0;
unsigned char set_Encoder1Byte4=0;
unsigned char set_Encoder2Byte1=0;
unsigned char set_Encoder2Byte2=0;
unsigned char set_Encoder2Byte3=0;
unsigned char set_Encoder2Byte4=0;
char input;											// speichert eingegebene Kommandos

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

int main(int argc, char **argv){
	// Announce this program to the ROS master as a "node" called "hello_world_node"
	ros::init(argc, argv, "md49_console_node");
	// Start the node resource managers (communication, time, etc)
	ros::start();
	// Broadcast a simple log message
	ROS_INFO_STREAM("md49_console started.");   
// Process ROS callbacks until receiving a SIGINT (ctrl-c)
  ros::spin();
  // Stop the node's resources
  ros::shutdown();	
filedesc = openSerialPort("/dev/ttyAMA0", B38400);
   	if (filedesc == -1) exit(1);
   	usleep(40000);									// Sleep for UART to power up and set options

  	printf("MD49_Console started \n");
   	print_help();

  	while(1){
	  printf("Comand:");
	  scanf("%s",&input);
	  parse_input();
	  if (input==113)
	  {								// "q" = quit programm
		close(fd);									// Close port
		return 0;									// exit
	  }
    }//end.mainloop
}//end.mainfunction

void parse_input (void){
	if (input==119){								// input="w" = Full Forward
		set_Speed1=255;
		set_Speed2=255;
		set_MD49_Commands();
	}
	if (input==120){								// "x" = Stop
		set_Speed1=128;
		set_Speed2=128;
		set_MD49_Commands();
	}
	if (input==97){									// "a" = Full Left
		set_Speed1=0;
		set_Speed2=255;
		set_MD49_Commands();
	}
	if (input==100){								// "d" = Full Right
		set_Speed1=255;
		set_Speed2=0;
		set_MD49_Commands();
	}
	if (input==115){								// "s" = Full Backward
		set_Speed1=0;
		set_Speed2=0;
		set_MD49_Commands();
	}
	if (input==104){								// "h" = print help again
		print_help();
	}
}

void print_help(void){
	printf("Console program to remote control robot hardware functions:\n");
	printf("---------------------------------------------------------- \n");
	printf("Enter 'w', 'a', 's', 'd'\n");
	printf("to drive forward, left, right, backward full speed. \n");
	printf("Enter 'x' to stop drives \n");
	printf("---------------------------------------------------------- \n");
	printf("Enter 'q' to quit program \n");
	printf("Enter 'h' to show instructions again \n");
	printf("---------------------------------------------------------- \n");
}

void set_MD49_Commands (void){
	serialBuffer[0] = 84;							// 84=T Steuerbyte um alle Commands an MD49 zu senden
	serialBuffer[1] = set_Speed1;					// speed1
	serialBuffer[2] = set_Speed2;					// speed2
	serialBuffer[3] = set_Acceleration;				// Acceleration
	serialBuffer[4] = set_Mode;						// Mode
	serialBuffer[5] = set_Reset;					// reset
	serialBuffer[6] = set_Regulator;				// Regulator
	serialBuffer[7] = set_Timeout;					// Timeout
	serialBuffer[8] = set_Encoder1Byte1;
	serialBuffer[9] = set_Encoder1Byte2;
	serialBuffer[10] = set_Encoder1Byte3;
	serialBuffer[11] = set_Encoder1Byte4;
	serialBuffer[12] = set_Encoder2Byte1;
	serialBuffer[13] = set_Encoder2Byte2;
	serialBuffer[14] = set_Encoder2Byte3;
	serialBuffer[15] = set_Encoder2Byte4;
	writeBytes(fd, 16);
}
