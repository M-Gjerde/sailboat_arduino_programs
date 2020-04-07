/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

// Use the following line if you have a Leonardo or MKR1000
//#define USE_USBCON

#define GPSSerial Serial1

#include "ros.h"
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>




ros::NodeHandle nh;
std_msgs::String str_msg;
std_msgs::String gps_msg;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher gps("gps", &gps_msg);
char hello[13] = "hello world!";
char gps_data[3] = "123";

//Prototypes
void sailboat_cmd_callback( const geometry_msgs::Twist msg){

    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    

    gps.publish(&gps_msg);
};


void setup() {
    // wait for hardware serial to appear
    while (!Serial);
    ros::Subscriber<geometry_msgs::Twist> sub("sailboat_cmd", &sailboat_cmd_callback);

    pinMode(LED_BUILTIN, OUTPUT);

    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(gps);
    nh.subscribe(sub);
    nh.getHardware()->setBaud(115200);

    // make this baud rate fast enough to we aren't waiting on it
    Serial.begin(115200);

    // 9600 baud is the default rate for the Ultimate GPS
    GPSSerial.begin(9600);
}


void loop() {

    str_msg.data = hello;
    chatter.publish(&str_msg);
    nh.spinOnce();
    delay(1);

}


void sendGPSData(){

    gps_msg.data = (char)GPSSerial.available();
    if (GPSSerial.available() > 32) {
        gps_msg.data = GPSSerial.read();
        gps.publish(&gps_msg);
    }
    gps.publish(&gps_msg);


}


void receiveROSCommands(){


}
