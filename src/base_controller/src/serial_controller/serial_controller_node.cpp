#include <iostream>                                         /* allows to perform standard input and output operations */
#include <fstream>
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <errno.h>                                          /* Error number definitions */
#include <termios.h>                                        /* POSIX terminal control definitions */
#include <ctype.h>                                        /* isxxx() */


const char* serialport="/dev/ttyAMA0";                      /* defines used serialport */
int serialport_bps=B38400;                                  /* defines baudrate od serialport */
int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
unsigned char speed_l=128, speed_r=128;                               /* speed to set for MD49 */

int filedesc;                                               // File descriptor of serial port we will talk to
int fd;                                                     /* serial port file descriptor */
unsigned char serialBuffer[16];                             /* Serial buffer to store uart data */
struct termios orig;                                        // Port options

using namespace std;

int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void read_MD49_Data (void);
void set_MD49_speed (unsigned char speed_l, unsigned char speed_r);

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

int main( int argc, char* argv[] ){

    // Open serial port
    // ****************
    filedesc = openSerialPort("/dev/ttyAMA0", serialport_bps);
    if (filedesc == -1) exit(1);
    usleep(10000);                                      // Sleep for UART to power up and set options



    while( 1 )
    {

        // Read encoder and other data from MD49
        // and put into sqlite db
        // *************************************
        read_MD49_Data();
        usleep(250000);
        // Read commands from sqlite db and
        // set speed and other commands to MD49
        // ************************************
        string line;
        ifstream myfile ("md49_commands.txt");
        if (myfile.is_open())
        {
            while ( getline (myfile,line) )
            {
                //cout << line << '\n';
                char data[10];
                std::copy(line.begin(), line.end(), data);
                speed_l=atoi(data);
            }
            myfile.close();
        }
        else cout << "Unable to open file";

    }// end.mainloop

    return 1;
} // end.main

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
      tcgetattr(fd, &orig); 						/* save current serial settings */
      tcgetattr(fd, &neu);
      cfmakeraw(&neu);
      //fprintf(stderr, "speed=%d\n", bps);
      cfsetispeed(&neu, bps);
      cfsetospeed(&neu, bps);
      tcflush(fd, TCIFLUSH);
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
    //Daten lesen und in Array schreiben
    readBytes(fd, 18);

    ofstream myfile;
    myfile.open ("md49_data.txt");
    //myfile << "Writing this to a file.\n";

    char buffer[33];

    printf("\033[2J");        /*  clear the screen  */
    printf("\033[H");         /*  position cursor at top-left corner */
    printf ("MD49-Data read from AVR-Master: \n");
    printf("====================================================== \n");
    printf("Encoder1 Byte1: %i ",serialBuffer[0]);
    myfile << itoa(serialBuffer[0],buffer,10);
    myfile << "\n";
    printf("Byte2: %i ",serialBuffer[1]);
    myfile << itoa(serialBuffer[1],buffer,10);
    myfile << "\n";
    printf("Byte3: % i ",serialBuffer[2]);
    myfile <<  itoa(serialBuffer[2],buffer,10);
    myfile << "\n";
    printf("Byte4: %i \n",serialBuffer[3]);
    myfile <<  itoa(serialBuffer[3],buffer,10);
    myfile << "\n";
    printf("Encoder2 Byte1: %i ",serialBuffer[4]);
    myfile <<  itoa(serialBuffer[4],buffer,10);
    myfile << "\n";
    printf("Byte2: %i ",serialBuffer[5]);
    myfile <<  itoa(serialBuffer[5],buffer,10);
    myfile << "\n";
    printf("Byte3: %i ",serialBuffer[6]);
    myfile <<  itoa(serialBuffer[6],buffer,10);
    myfile << "\n";
    printf("Byte4: %i \n",serialBuffer[7]);
    myfile <<  itoa(serialBuffer[7],buffer,10);
    myfile << "\n";
    printf("EncoderL: %i ",EncoderL);
    myfile <<  itoa(EncoderL,buffer,10);
    myfile << "\n";
    printf("EncoderR: %i \n",EncoderR);
    myfile <<  itoa(EncoderR,buffer,10);
    myfile << "\n";
    printf("====================================================== \n");
    printf("Speed1: %i ",serialBuffer[8]);
    myfile << itoa(serialBuffer[8],buffer,10);
    myfile << "\n";
    printf("Speed2: %i \n",serialBuffer[9]);
    myfile << itoa(serialBuffer[9],buffer,10);
    myfile << "\n";
    printf("Volts: %i \n",serialBuffer[10]);
    myfile << itoa(serialBuffer[10],buffer,10);
    myfile << "\n";
    printf("Current1: %i ",serialBuffer[11]);
    myfile << itoa(serialBuffer[11],buffer,10);
    myfile << "\n";
    printf("Current2: %i \n",serialBuffer[12]);
    myfile << itoa(serialBuffer[12],buffer,10);
    myfile << "\n";
    printf("Error: %i \n",serialBuffer[13]);
    myfile << itoa(serialBuffer[13],buffer,10);
    myfile << "\n";
    printf("Acceleration: %i \n",serialBuffer[14]);
    myfile << itoa(serialBuffer[14],buffer,10);
    myfile << "\n";
    printf("Mode: %i \n",serialBuffer[15]);
    myfile << itoa(serialBuffer[15],buffer,10);
    myfile << "\n";
    printf("Regulator: %i \n",serialBuffer[16]);
    myfile << itoa(serialBuffer[16],buffer,10);
    myfile << "\n";
    printf("Timeout: %i \n",serialBuffer[17]);
    myfile << itoa(serialBuffer[17],buffer,10);
    myfile << "\n";
    printf("speed_l = %i \n",speed_l);

    EncoderL = serialBuffer[0] << 24;                        // Put together first encoder value
    EncoderL |= (serialBuffer[1] << 16);
    EncoderL |= (serialBuffer[2] << 8);
    EncoderL |= (serialBuffer[3]);
    EncoderR = serialBuffer[4] << 24;                        // Put together second encoder value
    EncoderR |= (serialBuffer[5] << 16);
    EncoderR |= (serialBuffer[6] << 8);
    EncoderR |= (serialBuffer[7]);

    myfile.close();
}

void set_MD49_speed (unsigned char speed_l, unsigned char speed_r){
    serialBuffer[0] = 88;					// 88 =X Steuerbyte um Commands an MD49 zu senden
    serialBuffer[1] = 115;					// 115=s Steuerbyte setSpeed
    serialBuffer[2] = speed_l;				// speed1
    serialBuffer[3] = speed_r;				// speed2
    writeBytes(fd, 4);
}
