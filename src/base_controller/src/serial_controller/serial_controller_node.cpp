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
//#include <fstream>                                          /* Input/output stream class to operate on files. */
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
//#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <errno.h>                                          /* Error number definitions */
#include <termios.h>                                        /* POSIX terminal control definitions */
//#include <ctype.h>                                          /* isxxx() */
//#include<ros/ros.h>
#include <sqlite3.h>


// Global variables
const char* serialport_name="//dev/ttyAMA0";                   /* defines used serialport on BPi. Use "/dev/ttyAMA0" for RPi*/
int serialport_bps=B9600;                                  /* defines used baudrate on serialport */
//int filedesc;                                             /* File descriptor of serial port we will talk to*/
int fd;                                                     /* serial port file descriptor */
int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
unsigned char speed_l=128, speed_r=128;                     /* speed to set for MD49 */
unsigned char last_speed_l=128, last_speed_r=128;           /* speed to set for MD49 */
unsigned char serialBuffer[16];                             /* Serial buffer to store uart data */
struct termios orig;                                        // backuped port options

// sqlite globals
sqlite3 *db;
char *zErrMsg = 0;
int  rc;
const char *sql;
const char* data = "Callback function called";

using namespace std;

// Declare functions
// *********************************************************
int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void open_sqlite_db_md49data(void);
static int sql_callback(void *data, int argc, char **argv, char **azColName);
void read_MD49_Data_serial (void);
void read_md49_commands(void);
void set_MD49_speed (unsigned char speed_l, unsigned char speed_r);



int main( int argc, char* argv[] ){

    // Init as ROS node
    // ****************
    //ros::init(argc, argv, "serial_controller");
    //ros::NodeHandle n;

    // Open sqlite database md49data.db
    // ********************************
    open_sqlite_db_md49data();

    // Open serial port
    // ****************
    fd = openSerialPort(serialport_name, serialport_bps);
    if (fd == -1) exit(1);
    //ROS_INFO("Opend serial port at %s with %i Bps",serialport_name,serialport_bps);
    usleep(10000);                                          // Sleep for UART to power up and set options

    //ROS_DEBUG("Starting Mainloop...");
    //ROS_DEBUG("reading data from MD49 and pushing commands to MD49 @ 5Hz...");


    // Mainloop
    // ***********************************
    //while(  n.ok() ){
    while( 1 ){
        // Read encodervalues and other data from MD49
        // serial. Data ist stored in md49_data.txt
        // ****************************************
        read_MD49_Data_serial();
        usleep(100000);

        // Read commands from md49_commands.txt:
        // *************************************
        read_md49_commands();

        // Set speed and other commands as
        // read from md49_commands.txt
        // *******************************
        if ((speed_l != last_speed_l) || (speed_r != last_speed_r)){
            set_MD49_speed(speed_l, speed_r);
            last_speed_l=speed_l;
            last_speed_r=speed_r;
        }

        usleep(100000);

    }// end.mainloop
    sqlite3_close(db);
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

    // Write data read from MD49 into
    // sqlite3 database md49data.db
    // ******************************
    char sql_buffer[400];
    int cx;
    // EncoderL & EncoderR
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET EncoderL=%i, EncoderR=%i WHERE ID=1;", EncoderL, EncoderR);
    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Operation done successfully\n");
    }
    // Encoderbytes 1-4 left
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET Encoderbyte1L=%i, Encoderbyte2L=%i, " \
                   "Encoderbyte3L=%i, Encoderbyte4L=%i WHERE ID=1;", serialBuffer[0], serialBuffer[1], serialBuffer[2], serialBuffer[3]);
    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Operation done successfully\n");
    }
    // Encoderbytes 1-4 right
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET Encoderbyte1R=%i, Encoderbyte2R=%i, " \
                   "Encoderbyte3R=%i, Encoderbyte4R%i WHERE ID=1;", serialBuffer[4], serialBuffer[5], serialBuffer[6], serialBuffer[7]);
    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Operation done successfully\n");
    }

    // Output MD49 data on screen
    // **************************
    printf("\033[2J");                                      //  clear the screen
    printf("\033[H");                                       //  position cursor at top-left corner
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
    printf("SpeedL: %i ",serialBuffer[8]);
    printf("SpeedR: %i \n",serialBuffer[9]);
    printf("Volts: %i \n",serialBuffer[10]);
    printf("CurrentL: %i ",serialBuffer[11]);
    printf("CurrentR: %i \n",serialBuffer[12]);
    printf("Error: %i \n",serialBuffer[13]);
    printf("Acceleration: %i \n",serialBuffer[14]);
    printf("Mode: %i \n",serialBuffer[15]);
    printf("Regulator: %i \n",serialBuffer[16]);
    printf("Timeout: %i \n",serialBuffer[17]);

}

// Write serial command to change left and right speed
// ***************************************************
void set_MD49_speed (unsigned char speed_l, unsigned char speed_r){

    serialBuffer[0] = 88;                                   // 88 =X Steuerbyte um Commands an MD49 zu senden
    serialBuffer[1] = 115;                                  // 115=s Steuerbyte setSpeed
    serialBuffer[2] = speed_l;                              // set speed1
    serialBuffer[3] = speed_r;                              // set speed2
    writeBytes(fd, 4);
}

// Read SpeedL and SpeedR from
// table md49commands(md49data.db)
// *******************************
void read_md49_commands(void){

   // Create SQL statement
   sql = "SELECT * from md49commands WHERE ID=1";

   // Execute SQL statement
   rc = sqlite3_exec(db, sql, sql_callback, (void*)data, &zErrMsg);
   if( rc != SQLITE_OK ){
      fprintf(stderr, "SQL message: %s\n", zErrMsg);
      sqlite3_free(zErrMsg);
   }else{
      //fprintf(stdout, "Query done successfully\n");
   }
}

// Open serialport
// ***************
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

// Write bytes serial to UART
// **************************
void writeBytes(int descriptor, int count) {
    if ((write(descriptor, serialBuffer, count)) == -1) {   // Send data out
        perror("Error writing");
        close(descriptor);                                  // Close port if there is an error
        exit(1);
    }

}

// Read bytes serial from UART
// ***************************
void readBytes(int descriptor, int count) {
    if (read(descriptor, serialBuffer, count) == -1) {      // Read back data into buf[]
        perror("Error reading ");
        close(descriptor);                                  // Close port if there is an error
        exit(1);
    }
}

// Open sqlite db md49data.db and create
// table md49data if not existing
// *************************************
void open_sqlite_db_md49data(void){

    // Open database md49data.db and add table md49data
    // ************************************************
    rc = sqlite3_open("data/md49data.db", &db);
    if( rc ){
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        exit(0);
    }else{
        fprintf(stdout, "Opened database successfully,\n");
    }

    // Create table md49data
    // *********************
    sql = "CREATE TABLE md49data("  \
     "ID INT PRIMARY KEY     NOT NULL," \
     "Encoderbyte1L  INT DEFAULT 0, Encoderbyte2L  INT DEFAULT 0," \
     "Encoderbyte3L  INT DEFAULT 0, Encoderbyte4L  INT DEFAULT 0," \
     "Encoderbyte1R  INT DEFAULT 0, Encoderbyte2R  INT DEFAULT 0," \
     "Encoderbyte3R  INT DEFAULT 0, Encoderbyte4R  INT DEFAULT 0," \
     "EncoderL       INT DEFAULT 0, EncoderR       INT DEFAULT 0," \
     "SpeedL         INT DEFAULT 0, SpeedR         INT DEFAULT 0," \
     "Volts          INT DEFAULT 0," \
     "CurrentL       INT DEFAULT 0, CurrentR       INT DEFAULT 0," \
     "Error          INT DEFAULT 0, Acceleration   INT DEFAULT 0," \
     "Mode           INT DEFAULT 0, Regulator      INT DEFAULT 0," \
     "Timeout        INT DEFAULT 0 );" \
     "INSERT INTO md49data (ID) VALUES (1);";

    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        fprintf(stdout, "table created successfully\n");
    }
}

// Callbackfunction executed if table md49commands in md49data.db is queried
// *************************************************************************
static int sql_callback(void *data, int argc, char **argv, char **azColName){
   speed_l= atoi(argv[1]);
   speed_r= atoi(argv[2]);
   return 0;
}


