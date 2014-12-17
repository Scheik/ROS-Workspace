/*! \brief serial_controller_node.
 *         Manages serial communications with AVR-Master.
 *
 *  This node manages serial communications with the AVR-Master.
 *  Therefore it first requests all data from AVR-Master and then sends all
 *  commands to AVR-Master in a Loop. Data read is stored in md49_data.txt,
 *  commands to be send are read from md49_commands.txt.
 *
 */

// Includes
// *********************************************************
#include <iostream>                                         /* allows to perform standard input and output operations */
#include <fstream>                                          /* Input/output stream class to operate on files. */
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <errno.h>                                          /* Error number definitions */
#include <termios.h>                                        /* POSIX terminal control definitions */
#include <ctype.h>                                          /* isxxx() */
//#include<ros/ros.h>

// Global variables
const char* serialport_name="/dev/ttyS2";                 /* defines used serialport on BPi. Use "/dev/ttyAMA0" for RPi*/
int serialport_bps=B38400;                                  /* defines used baudrate on serialport */
//int filedesc;                                             /* File descriptor of serial port we will talk to*/
int fd;                                                     /* serial port file descriptor */
int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
unsigned char speed_l=128, speed_r=128;                     /* speed to set for MD49 */
unsigned char last_speed_l=128, last_speed_r=128;                     /* speed to set for MD49 */
unsigned char serialBuffer[16];                             /* Serial buffer to store uart data */
unsigned char md49_data[18];                                /* keeps data from MD49, read from AVR-Master */
struct termios orig;                                        // Port options

using namespace std;

// Declare functions
// *********************************************************
int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void read_MD49_Data_serial (void);
void read_md49_commands_txt(void);
void set_MD49_speed (unsigned char speed_l, unsigned char speed_r);
char* itoa(int value, char* result, int base);


int main( int argc, char* argv[] ){

    // Init as ROS node
    // ****************
    //ros::init(argc, argv, "serial_controller");
    //ros::NodeHandle n;

    // Open serial port
    // ****************
    //fd = openSerialPort("/dev/ttyAMA0", serialport_bps);    // RPis UART from GPIO header
    fd = openSerialPort(serialport_name, serialport_bps);    // RPis UART from GPIO header
    if (fd == -1) exit(1);
    //ROS_INFO("Opend serial port at %s with %i Bps",serialport_name,serialport_bps);
    usleep(10000);                                          // Sleep for UART to power up and set options

    //ROS_DEBUG("Starting Mainloop...");
    //ROS_DEBUG("reading data from MD49 and pushing commands to MD49 @ 5Hz...");

    //while(  n.ok() ){
    while( 1 ){
        // Read encoder and other data from MD49
        // serial. Data ist stored in md49_data.txt
        // ****************************************
        read_MD49_Data_serial();
        usleep(100000);

        // Read commands from md49_commands.txt:
        // *************************************
        read_md49_commands_txt();

        // Set speed and other commands as
        // read from md49_commands.txt
        // *******************************
        set_MD49_speed(speed_l, speed_r);
        usleep(100000);

    }// end.mainloop
    return 1;
} // end.main

void read_MD49_Data_serial (void){
    // Read serial MD49 data from AVR-Master
    // *************************************
    serialBuffer[0] = 82;                                   // 82=R Steuerbyte um alle Daten vom MD49 zu lesen
    writeBytes(fd, 1);
    readBytes(fd, 18);    

    // Put toghether encoder values from their
    // corresponding bytes, read from MD49
    // ***************************************
    EncoderL = serialBuffer[0] << 24;                       // Put together first encoder value
    EncoderL |= (serialBuffer[1] << 16);
    EncoderL |= (serialBuffer[2] << 8);
    EncoderL |= (serialBuffer[3]);
    EncoderR = serialBuffer[4] << 24;                       // Put together second encoder value
    EncoderR |= (serialBuffer[5] << 16);
    EncoderR |= (serialBuffer[6] << 8);
    EncoderR |= (serialBuffer[7]);

    // Write data from MD49 into md49_data.txt
    // ***************************************
    int i=0;
    char buffer[33];
    ofstream myfile;
    myfile.open ("md49_data.txt");
    //myfile << "Writing this to a file.\n";
    for (i=0;i<18;i++){
        if (serialBuffer[i]==0){
            myfile << "000";
        }
        else if (serialBuffer[i]<10){
            myfile << "00";
            myfile << itoa(serialBuffer[i],buffer,10);
        }
        else if (serialBuffer[i]<100){
            myfile << "0";
            myfile << itoa(serialBuffer[i],buffer,10);
        }
        else{
            myfile << itoa(serialBuffer[i],buffer,10);
        }
        myfile << "\n";
    }
    myfile.close();

    // Output MD49 data on screen
    // **************************
    printf("\033[2J");                                      /*  clear the screen  */
    printf("\033[H");                                       /*  position cursor at top-left corner */
    printf ("MD49-Data read from AVR-Master: \n");
    printf("========================================\n");
    printf("Encoder1 Byte1: %i ",serialBuffer[0]);
    printf("Byte2: %i ",serialBuffer[1]);
    printf("Byte3: % i ",serialBuffer[2]);
    printf("Byte4: %i \n",serialBuffer[3]);
    printf("Encoder2 Byte1: %i ",serialBuffer[4]);
    printf("Byte2: %i ",serialBuffer[5]);
    printf("Byte3: %i ",serialBuffer[6]);
    printf("Byte4: %i \n",serialBuffer[7]);
    printf("EncoderL: %i ",EncoderL);
    printf("EncoderR: %i \n",EncoderR);
    printf("========================================\n");
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

void set_MD49_speed (unsigned char speed_l, unsigned char speed_r){
    serialBuffer[0] = 88;                                   // 88 =X Steuerbyte um Commands an MD49 zu senden
    serialBuffer[1] = 115;                                  // 115=s Steuerbyte setSpeed
    serialBuffer[2] = speed_l;                              // set speed1
    serialBuffer[3] = speed_r;                              // set speed2
    writeBytes(fd, 4);
}

void read_md49_commands_txt(void){
    string line;
    ifstream myfile ("md49_commands.txt");
    if (myfile.is_open())
    {
        int i=0;
        while ( getline (myfile,line) )
        {
            //cout << line << '\n';
            char data[10];
            std::copy(line.begin(), line.end(), data);
            md49_data[i]=atoi(data);
            i =i++;
        }
        myfile.close();
        speed_l=md49_data[0];
        speed_r=md49_data[1];
    }
    else cout << "Unable to open file";
}

int openSerialPort(const char * device, int bps){
   struct termios neu;
   char buf[128];

   //fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
   fd = open(device, O_RDWR | O_NOCTTY);

   if (fd == -1)
   {
      sprintf(buf, "openSerialPort %s error", device);
      perror(buf);
   }
   else
   {
      tcgetattr(fd, &orig);                                 /* save current serial settings */
      tcgetattr(fd, &neu);
      cfmakeraw(&neu);
      cfsetispeed(&neu, bps);
      cfsetospeed(&neu, bps);
      tcflush(fd, TCIFLUSH);
      tcsetattr(fd, TCSANOW, &neu);                         /* set new serial settings */
      fcntl (fd, F_SETFL, O_RDWR);
   }
   return fd;
}

void writeBytes(int descriptor, int count) {
    if ((write(descriptor, serialBuffer, count)) == -1) {   // Send data out
        perror("Error writing");
        close(descriptor);                                  // Close port if there is an error
        exit(1);
    }

}

void readBytes(int descriptor, int count) {
    if (read(descriptor, serialBuffer, count) == -1) {      // Read back data into buf[]
        perror("Error reading ");
        close(descriptor);                                  // Close port if there is an error
        exit(1);
    }
}

char* itoa(int value, char* result, int base) {
        // check that the base if valid
        if (base < 2 || base > 36) { *result = '\0'; return result; }

        char* ptr = result, *ptr1 = result, tmp_char;
        int tmp_value;

        do {
            tmp_value = value;
            value /= base;
            *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * base)];
        } while ( value );

        // Apply negative sign
        if (tmp_value < 0) *ptr++ = '-';
        *ptr-- = '\0';
        while(ptr1 < ptr) {
            tmp_char = *ptr;
            *ptr--= *ptr1;
            *ptr1++ = tmp_char;
        }
        return result;
    }