#include <iostream>                                         /* allows to perform standard input and output operations */
#include <stdio.h>                                          /* Standard input/output definitions */
#include <stdint.h>                                         /* Standard input/output definitions */
#include <stdlib.h>                                         /* defines several general purpose functions */
#include <unistd.h>                                         /* UNIX standard function definitions */
#include <fcntl.h>                                          /* File control definitions */
#include <ctype.h>                                          /* isxxx() */
#include <sqlite3.h>
#include <ros/ros.h>                                        /* ROS */
#include <base_controller/encoders.h>                       /* Custom message /encoders */
#include <base_controller/md49data.h>                       /* Custom message /encoders */

base_controller::encoders encoders;
base_controller::md49data md49data;

// sqlite globals
sqlite3 *db;
char *zErrMsg = 0;
int  rc;
int cx;
const char *sql;
char sql_buffer[400];

void open_sqlite_db_md49data(void);
void execute_update_sqlite(void);

void encoders_callback(const base_controller::encoders& encoders){

    // Write data read from MD49 into
    // sqlite3 database md49data.db
    // ******************************
    // EncoderL & EncoderR
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET EncoderL=%i, EncoderR=%i WHERE ID=1;", encoders.encoder_l, encoders.encoder_r);
    execute_update_sqlite();
    // Encoderbytes 1-4 left
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET Encoderbyte1L=%i, Encoderbyte2L=%i, " \
     "Encoderbyte3L=%i, Encoderbyte4L=%i WHERE ID=1;", encoders.encoderbyte1l, encoders.encoderbyte2l, encoders.encoderbyte3l, encoders.encoderbyte4l);
    execute_update_sqlite();
    // Encoderbytes 1-4 right
    cx = snprintf (sql_buffer,400,"UPDATE md49data SET Encoderbyte1R=%i, Encoderbyte2R=%i, " \
     "Encoderbyte3R=%i, Encoderbyte4R=%i WHERE ID=1;", encoders.encoderbyte1r, encoders.encoderbyte2r, encoders.encoderbyte3r, encoders.encoderbyte4r);
    execute_update_sqlite();
}

void md49data_callback(const base_controller::md49data& md49data){
        // SpeedL, SpeedR, Volts, CurrentL, CurrentR, Error, Acceleration, Mode, Regulator, Timeout
        cx = snprintf (sql_buffer,400,"UPDATE md49data SET SpeedL=%i, SpeedR=%i, " \
                       "Volts=%i, CurrentL=%i, CurrentR=%i, Error=%i, Acceleration=%i, Mode=%i, " \
                       "Regulator=%i, Timeout=%i " \
                       "WHERE ID=1;",md49data.speed_l,md49data.speed_r,md49data.volts,md49data.current_l,md49data.current_r,md49data.error,md49data.acceleration,md49data.mode,md49data.regulator,md49data.timeout);
        execute_update_sqlite();

}

int main( int argc, char* argv[] ){

    // Init node
    // *********
    ros::init(argc, argv, "sqlite_controller" );
    ros::NodeHandle n;
    ros::Subscriber encoders_sub = n.subscribe("/encoders", 10, encoders_callback);
    ros::Subscriber md49data_sub = n.subscribe("/md49data", 10, md49data_callback);
    ros::Rate loop_rate(2);
    ROS_INFO("sqlite_connector running...");
    ROS_INFO("=============================");
    ROS_INFO("Subscribing to topic /encoders");
    ROS_INFO("Subscribing to topic /md49data");

    // Open sqlite database md49data.db
    // ********************************
    open_sqlite_db_md49data();
    ROS_INFO("Opend md49data.db database");

    // Mainloop
    // ********
    while(n.ok()){

        ros::spinOnce();
        loop_rate.sleep();

    }// end.mainloop
    sqlite3_close(db);
    return 1;
}// end.main


// Open sqlite db md49data.db and create
// table md49data if not existing
// *************************************
void open_sqlite_db_md49data(void){

    // Open database md49data.db and add table md49data
    // ************************************************
    rc = sqlite3_open("data/md49data.db", &db);
    if( rc ){
        ROS_WARN("Can't open database md49data.db: %s,", sqlite3_errmsg(db));
        //fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        exit(0);
    }else{
        ROS_INFO("Opened database md49data.db successfully,");
        //fprintf(stdout, "Opened database successfully,\n");
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
        ROS_WARN("SQL message: %s", zErrMsg);
        //fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        ROS_INFO("table md49data created successfully");
        //fprintf(stdout, "table created successfully\n");
    }
}

void execute_update_sqlite(void){
    rc = sqlite3_exec(db, sql_buffer, NULL, 0, &zErrMsg);
    if( rc != SQLITE_OK ){
        ROS_WARN("SQL message: %s", zErrMsg);
        //fprintf(stderr, "SQL message: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }else{
        //fprintf(stdout, "Operation done successfully\n");
    }
}
