#include <iostream>                                         /* allows to perform standard input and output operations */
#include <fstream>
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <ctype.h>                                          /* isxxx() */
#include <ros/ros.h>                                        /* ROS */
#include <geometry_msgs/Twist.h>                            /* ROS Twist message */
#include <base_controller/encoders.h>                       /* Custom message /encoders */
#include <base_controller/md49data.h>                       /* Custom message /encoders */
#include <sqlite3.h>

int32_t EncoderL;                                           /* stores encoder value left read from md49 */
int32_t EncoderR;                                           /* stores encoder value right read from md49 */
unsigned char speed_l=128, speed_r=128;                     /* speed to set for MD49 */
unsigned char last_speed_l=128, last_speed_r=128;           /* speed to set for MD49 */
double vr = 0.0;
double vl = 0.0;
double max_vr = 0.2;
double max_vl = 0.2;
double min_vr = 0.2;
double min_vl = 0.2;
double base_width = 0.4;                                    /* Base width in meters */

unsigned char serialBuffer[18];                             /* Serial buffer to store uart data */
void read_MD49_Data (void);
void set_md49_speed (unsigned char speed_l, unsigned char speed_r);
void open_sql_db_md49data(void);

// sqlite globals
sqlite3 *db;
char *zErrMsg = 0;
int  rc;
const char *sql;
const char* data = "Callback function called";

using namespace std;

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

void open_sql_db_md49data(void){
    // Open database md49data.db and add
    // table md49commands
    // *********************************
    rc = sqlite3_open("data/md49data.db", &db);
    if( rc ){
        ROS_INFO("Can't open database: %s", sqlite3_errmsg(db));
        exit(0);
    }else{
        ROS_INFO("Opened database successfully,");
    }

    // Create table md49commands
    // *************************
    sql = "CREATE TABLE md49commands("  \
     "ID INT PRIMARY KEY     NOT NULL," \
     "SpeedL         INT DEFAULT 128," \
     "SpeedR         INT DEFAULT 128 );" \
     "INSERT INTO md49commands (ID,SpeedL,SpeedR) VALUES (1,128,128);";
    rc = sqlite3_exec(db, sql, NULL, 0, &zErrMsg);                      // Execute SQL statement
    if( rc != SQLITE_OK ){
        ROS_INFO("SQL message2: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        ROS_INFO("table created successfully");
    }

    // Set SpeedL and SpeedR to
    // defaults =128
    // ************************
    char sql_buffer[200];
    int cx;
    cx = snprintf (sql_buffer,200,"UPDATE md49commands SET SpeedL=%i, SpeedR=%i WHERE ID=1", 128,128);

    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        ROS_INFO("SQL message: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        ROS_INFO("SpeedL & SpeedR set to defaults in Table md49commands(md49data.db)");
    }

}


int main( int argc, char* argv[] ){

    // Setup as ROS node
    // *****************
    ros::init(argc, argv, "base_controller" );
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);
    ros::Publisher encoders_pub = n.advertise<base_controller::encoders>("encoders",10);
    ros::Publisher md49data_pub = n.advertise<base_controller::md49data>("md49data",10);

    ros::Rate loop_rate(10);
    ROS_INFO("Starting base_controller node:");
    ROS_INFO("============================================");
    ROS_INFO("subscribing to /cmd_vel");
    ROS_INFO("publishing to /encoders");
    ROS_INFO("publishing to /md49data");
    ROS_INFO("============================================");

    open_sql_db_md49data();

    while(n.ok())
    {
        // Read encoder values and other data from MD49:
        // serial_controller_node reads data from AVR-Master
        // and provides that data in md49_data.txt
        // *************************************************
//        read_MD49_Data();

        // Set MD49 speed_l and speed_r:
        // serial_controller_node reads commands from
        // md49_commands.txt and writes commands to AVR-Master
        // ***************************************************
        if ((speed_l != last_speed_l) || (speed_r != last_speed_r)){
            set_md49_speed(speed_l,speed_r);
            last_speed_l=speed_l;
            last_speed_r=speed_r;
        }

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

        ros::spinOnce();
        loop_rate.sleep();

    }// end.mainloop

    return 1;
} // end.main


void read_MD49_Data (void){

    // Read MD49 data from md49data.db
    // *******************************
    // code here!


    // Read all MD49 data from md49_data.txt
    // *************************************
    string line;
    ifstream myfile ("md49_data.txt");
    if (myfile.is_open()){
        int i=0;
        while (getline (myfile,line)){
            //cout << line << '\n';
            char data[10];
            std::copy(line.begin(), line.end(), data);
            serialBuffer[i]=atoi(data);
            i =i++;
        }
        myfile.close();

    }
    else ROS_ERROR("Unable to open file: md49_data.txt");

    // Put toghether new encodervalues
    // *******************************
    EncoderL = serialBuffer[0] << 24;                        // Put together first encoder value
    EncoderL |= (serialBuffer[1] << 16);
    EncoderL |= (serialBuffer[2] << 8);
    EncoderL |= (serialBuffer[3]);
    EncoderR = serialBuffer[4] << 24;                        // Put together second encoder value
    EncoderR |= (serialBuffer[5] << 16);
    EncoderR |= (serialBuffer[6] << 8);
    EncoderR |= (serialBuffer[7]);

}

void set_md49_speed (unsigned char speed_l, unsigned char speed_r){

    // Set SpeedL=speed_l and
    // SpeedR=speed_r in Table md49commands
    // ************************************
    char sql_buffer[200];
    int cx;
    cx = snprintf (sql_buffer,200,"UPDATE md49commands SET SpeedL=%i, SpeedR=%i WHERE ID=1", speed_l,speed_r);

    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        ROS_WARN("SQL message1: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        ROS_INFO("Set SpeedL=%i and SpeedR=%i in Table md49commands(md49data.db)",speed_l, speed_r);
    }
}


