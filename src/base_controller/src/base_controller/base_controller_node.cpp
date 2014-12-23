#include <iostream>                                         /* allows to perform standard input and output operations */
//#include <fstream>
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
//#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <ctype.h>                                          /* isxxx() */
#include <termios.h>                                        /* POSIX terminal control definitions */
#include <errno.h>                                          /* Error number definitions */
#include <sqlite3.h>
#include <ros/ros.h>                                        /* ROS */
#include <geometry_msgs/Twist.h>                            /* ROS Twist message */
#include <base_controller/encoders.h>                       /* Custom message /encoders */
#include <base_controller/md49data.h>                       /* Custom message /encoders */


const char* serialport_name="/dev/ttyS2";                   /* defines used serialport on BPi. Use "/dev/ttyAMA0" for RPi*/
int serialport_bps=B38400;                                  /* defines used baudrate on serialport */
int fd;                                                     /* serial port file descriptor */
struct termios orig;                                        // backuped port options
int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
char speed_l=128;
char speed_r=128;                                  /* speed to set for MD49 */
char last_speed_l=128, last_speed_r=128;           /* speed to set for MD49 */
double vr = 0.0;
double vl = 0.0;
double max_vr = 0.2;
double max_vl = 0.2;
double min_vr = 0.2;
double min_vl = 0.2;
double base_width = 0.4;                                    /* Base width in meters */

unsigned char serialBuffer[18];                             /* Serial buffer to store uart data */
void read_MD49_Data (void);
void set_MD49_speed (void);
char* itoa(int value, char* result, int base);

int openSerialPort(const char * device, int bps);
void writeBytes(int descriptor, int count);
void readBytes(int descriptor, int count);
void open_sqlite_db_md49data(void);

// sqlite globals
sqlite3 *db;
char *zErrMsg = 0;
int  rc;
const char *sql;
const char* data = "Callback function called";


base_controller::encoders encoders;
base_controller::md49data md49data;

void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd){

        if (vel_cmd.linear.x>0){
            speed_l = 255;
            speed_r = 255;
        }
        if (vel_cmd.linear.x<0){
            speed_l = 0;
            speed_r = 0;
        }
        if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){
            speed_l = 128;
            speed_r = 128;
        }
        if (vel_cmd.angular.z>0){
            speed_l = 0;
            speed_r = 255;
        }
        if (vel_cmd.angular.z<0){
            speed_l = 255;
            speed_r = 0;
        }


    /*
        //ANFANG Alternative
        if (vel_cmd.linear.x==0 && vel_cmd.angular.z==0){vl=0;vr=0;}
        else if(vel_cmd.linear.x == 0){
            // turning
            vr = vel_cmd.angular.z * base_width / 2.0;
            vl = (-1) * vr;
        }
        else if(vel_cmd.angular.z == 0){
            // forward / backward
            vl = vr = vel_cmd.linear.x;
        }
        else{
            // moving doing arcs
            vl = vel_cmd.linear.x - vel_cmd.angular.z * base_width / 2.0;
            if (vl > max_vl) {vl=max_vl;}
            if (vl < min_vl) {vl=min_vl;}
            vr = vel_cmd.linear.x + vel_cmd.angular.z * base_width / 2.0;
            if (vr > max_vr) {vr=max_vr;}
            if (vr < min_vr) {vr=min_vr;}
        }
        //ENDE Alternative
    */
}


int main( int argc, char* argv[] ){

    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<base_controller::encoders>("encoders",10);
    ros::Publisher md49data_pub = n.advertise<base_controller::md49data>("md49data",10);

    // Init node
    // *********
    ros::Rate loop_rate(25);
    ROS_INFO("base_controller running...");
    ROS_INFO("=============================");
    ROS_INFO("Subscribing to topic /cmd_vel");
    ROS_INFO("Publishing to topic /encoders");
    ROS_INFO("Publishing to topic /md49data");

    // Open serial port
    // ****************
    fd = openSerialPort(serialport_name, serialport_bps);
    if (fd == -1){
        ROS_FATAL("Could not open serialport at %s with %i",serialport_name,serialport_bps);
        exit(1);
    }
    ROS_INFO("Opend serial port at %s with %i Bps",serialport_name,serialport_bps);
    usleep(10000); // Sleep for UART to power up and set options

    // Open sqlite database md49data.db
    // ********************************
    open_sqlite_db_md49data();
    ROS_INFO("Opend md49data.db database");

    while(n.ok())
    {
        // Read encoder and other data from MD49 via UART
        // **********************************************
        read_MD49_Data();

        // set speed on MD49 via UART
        // **************************
        set_MD49_speed();

        // Publish encoder values to topic /encoders (custom message)
        // **********************************************************       
        encoders.encoder_l=EncoderL;
        encoders.encoder_r=EncoderR;
        encoders_pub.publish(encoders);

        // Publish MD49 data to topic /md49data (custom message)
        // *****************************************************        
        md49data.speed_l = serialBuffer[8];
        md49data.speed_r = serialBuffer[9];
        md49data.volt = serialBuffer[10];
        md49data.current_l = serialBuffer[11];
        md49data.current_r = serialBuffer[12];
        md49data.error = serialBuffer[13];
        md49data.acceleration = serialBuffer[14];
        md49data.mode = serialBuffer[15];
        md49data.regulator = serialBuffer[16];
        md49data.timeout = serialBuffer[17];
        md49data_pub.publish(md49data);

        // ************
        ros::spinOnce();
        loop_rate.sleep();

    }// end.mainloop
    return 1;
} // end.main


void read_MD49_Data (void){

    // Read serial MD49 encoder data from MD49
    // ***************************************
    serialBuffer[0] = 0;
    serialBuffer[1] = 0x25;					// Command to return encoder values
    writeBytes(fd, 2);
    readBytes(fd, 8);
    // Put toghether encoder values from their
    // corresponding bytes
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
                   "Encoderbyte3R=%i, Encoderbyte4R=%i WHERE ID=1;", serialBuffer[4], serialBuffer[5], serialBuffer[6], serialBuffer[7]);
    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Operation done successfully\n");
    }

/*
    // SpeedL, SpeedR, Volts, CurrentL, CurrentR, Error, Acceleration, Mode, Regulator, Timeout
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET SpeedL=%i, SpeedR=%i, " \
                   "Volts=%i, CurrentL=%i, CurrentR=%i, Error=%i, Acceleration=%i, Mode=%i, " \
                   "Regulator=%i, Timeout=%i " \
                   "WHERE ID=1;", serialBuffer[8], serialBuffer[9], serialBuffer[10], serialBuffer[11], serialBuffer[12], serialBuffer[13], serialBuffer[14], serialBuffer[15], serialBuffer[16], serialBuffer[17]);
    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Operation done successfully\n");
    }
*/
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
    //printf("SpeedL: %i ",serialBuffer[8]);
    //printf("SpeedR: %i \n",serialBuffer[9]);
    //printf("Volts: %i \n",serialBuffer[10]);
    //printf("CurrentL: %i ",serialBuffer[11]);
    //printf("CurrentR: %i \n",serialBuffer[12]);
    //printf("Error: %i \n",serialBuffer[13]);
    //printf("Acceleration: %i \n",serialBuffer[14]);
    //printf("Mode: %i \n",serialBuffer[15]);
    //printf("Regulator: %i \n",serialBuffer[16]);
    //printf("Timeout: %i \n",serialBuffer[17]);

}

void set_MD49_speed(void){

    serialBuffer[0] = 0;
    serialBuffer[1] = 0x31;					// Command to set motor speed
    serialBuffer[2] = speed_l;				// Speed to be set

    serialBuffer[3] = 0;
    serialBuffer[4] = 0x32;
    serialBuffer[5] = speed_r;

    writeBytes(fd, 6);
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
