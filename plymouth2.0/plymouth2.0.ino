/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON


#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#include <Adafruit_GPS.h>

#define GPSSerial Serial1
// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

uint32_t timer = millis();


//Prototypes
void sendGPSData();
void publishMsg(String msg);
void publishMsg(const char *msg);
void initPWMDriver(); //TODO renaming
void driveActuators(); //TODO renaming
void initGPS();

ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::String gps_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher gps("gps", &gps_msg);
char hello[13] = "hello world!";
char gps_data[4] = "123";

//Prototypes
void sailboat_cmd_callback(const geometry_msgs::Twist msg) {

    publishMsg(String(msg.angular.x));
    driveActuators();
};


void setup() {
    // wait for hardware serial to appear
    while (!Serial);
    ros::Subscriber <geometry_msgs::Twist> sub("sailboat_cmd", &sailboat_cmd_callback);

    pinMode(LED_BUILTIN, OUTPUT);

    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(gps);
    nh.subscribe(sub);
    nh.getHardware()->setBaud(115200);

    // make this baud rate fast enough to we aren't waiting on it
    Serial.begin(115200);

    /// GPS SETUP ///
    initGPS();

    /// MOTOR SHIELD SETUP ///
    initPWMDriver();


}


void loop() {
    sendGPSData();
    nh.spinOnce();
    delay(1);

}


void publishMsg(String msg) {
    char buf[256];
    msg.toCharArray(buf, 256);
    str_msg.data = buf;
    chatter.publish(&str_msg);

}

void publishMsg(const char *msg) {
    str_msg.data = msg;
    chatter.publish(&str_msg);
}

void initPWMDriver() {
    pwm.begin();
    pwm.setPWMFreq(60);  // This is the maximum PWM frequency
}

void driveActuators() {
    /*
    // Drive each PWM in a 'wave'
    for (uint16_t i = 0; i < 4096; i += 8) {
        for (uint8_t pwmnum = 0; pwmnum < 16; pwmnum++) {
            pwm.setPWM(pwmnum, 0, (i + (4096 / 16) * pwmnum) % 4096);
        }
    }
     */
}


void initGPS(){
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
    publishMsg("GPS firmware version: " + String(PMTK_Q_RELEASE));

}

void sendGPSData() {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
        if (c) Serial.print(c);
    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() fla$
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() fla$
            return; // we can fail to parse a sentence in which case we should just w$
    }


    if (millis() - timer > 2000) {
        timer = millis();

        if (GPS.fix) {
            publishMsg(String("incoming Data"));
            publishMsg("Time: " + String(GPS.hour + 2) + ":" + String(GPS.minute) + ":" +
                       String(GPS.seconds)); //Hour + 2 because summer time
            publishMsg("Date: " + String(GPS.day) + "/" + String(GPS.month) + "/" + String(GPS.year));
            publishMsg("Location: " + String(GPS.latitude, 4) + String(GPS.lat) + " " + String(GPS.longitude, 4) +
                       String(GPS.lon));
            publishMsg("Satellites: " + String(GPS.satellites));
            publishMsg("Quality: " + String(GPS.fixquality));
        } else {
            publishMsg("Gps was not fixed");
        }

    }
}