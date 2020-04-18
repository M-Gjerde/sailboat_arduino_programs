
#include "ros.h"
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_GPS.h>


//Initialize the pwm driver object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


#define rc_ch2 A11 //Channel used for sail
#define rc_ch1 A9  //channel used for rudder

#define GPSSerial Serial1 //Define which USART the GPS is using
#define GPSECHO false //Defined falso to not enter debugging mode
Adafruit_GPS GPS(&GPSSerial); //Pass Address of serial 1 to Adafruit GPS initalizer function
uint32_t timer = millis(); //Timer for GPS readings every two seconds.

//Prototypes
void sendGPSData();

void publishGPS(String msg);

//void publishMsg(const char *msg);
void publishRC(int msg);

void initPWMDriver(); //TODO renaming
void driveActuators(); //TODO renaming
void initGPS();

void readRadioController();

//ROS DEFINES
ros::NodeHandle nh;
std_msgs::String str_msg;
geometry_msgs::Twist gps_msg;
std_msgs::Int16 rc_msg;
ros::Publisher gps("gps", &str_msg);
ros::Publisher DebugPublisher("debug", &gps_msg);
ros::Publisher rc("rc", &rc_msg);


void sailboat_cmd_callback(const geometry_msgs::Twist msg) {
    driveActuators(msg.angular.x);
};

ros::Subscriber <geometry_msgs::Twist> rpi_commands("sailboat_cmd", &sailboat_cmd_callback);

void setup() {
    // wait for hardware serial to appear
    while (!Serial);

    pinMode(LED_BUILTIN, OUTPUT);

    nh.initNode();
    nh.advertise(gps);
    nh.advertise(DebugPublisher);
    nh.advertise(rc);
    nh.subscribe(rpi_commands);
    nh.getHardware()->setBaud(115200);
    Serial.begin(115200); //Begin Serialport for ROS to RPI communication

    /// GPS SETUP ///
    initGPS();

    /// MOTOR SHIELD SETUP ///
    initPWMDriver();

    pinMode(rc_ch2, INPUT);
    pinMode(rc_ch1, INPUT);

}


void loop() {
    sendGPSData();
    readRadioController();
    nh.spinOnce();
    delay(1);

}

void readRadioController() {
    int sail_channel = pulseIn(rc_ch2, HIGH);
    int rudder_channel = pulseIn(rc_ch1, HIGH);

    publishRC(sail_channel);
    publishRC(rudder_channel);
}

void publishGPS(String msg) {
    char buf[256];
    msg.toCharArray(buf, 256);
    str_msg.data = buf;
    gps.publish(&str_msg);
}

void publishRC(int msg) {
    rc_msg.data = msg;
    rc.publish(&rc_msg);
}

void initPWMDriver() {
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
}

void driveActuators(int x) {
    if (x > SERVOMIN && x < SERVOMAX) //If value to write to motors is between servo MAX and MIN
    pwm.setPWM(0, 0, x);


    //
    gps_msg.angular.x = x;
    DebugPublisher.publish(&gps_msg);
}


void initGPS() {
    // 9600 baud is the default rate for the Ultimate GPS
    GPS.begin(9600);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data$
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, a$
    // print it out we don't suggest using anything higher than 1 Hz
    // Request updates on antenna status, comment out to keep quiet
    //GPS.sendCommand(PGCMD_ANTENNA);
    delay(1000);
    // Ask for firmware version
    publishGPS("GPS firmware version: " + String(PMTK_Q_RELEASE));

}

void sendGPSData() {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO) //Debuging
        if (c);// publishMsg((String)c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        //publishMsg((String)GPS.lastNMEA()); // this also sets the newNMEAreceived() fla$
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() fla$
            return; // we can fail to parse a sentence in which case we should just w$
    }


    if (millis() - timer > 2000) {
        timer = millis();

        Serial.print("Fix: ");
        Serial.print((int) GPS.fix);
        Serial.print(" quality: ");
        Serial.println((int) GPS.fixquality);

        publishGPS("Fix: " + (String) GPS.fix + " quality: " + (String) GPS.fixquality);

        if (GPS.fix) {
            publishGPS(String("incoming Data"));
            publishGPS("Time: " + String(GPS.hour + 2) + ":" + String(GPS.minute) + ":" +
                       String(GPS.seconds)); //Hour + 2 because summer time
            publishGPS("Date: " + String(GPS.day) + "/" + String(GPS.month) + "/" + String(GPS.year));
            publishGPS("Location: " + String(GPS.latitude, 4) + String(GPS.lat) + " " + String(GPS.longitude, 4) +
                       String(GPS.lon));
            publishGPS("Satellites: " + String(GPS.satellites));
            publishGPS("Quality: " + String(GPS.fixquality));
        } else {
            publishGPS("Gps was not fixed");
        }

    }
}