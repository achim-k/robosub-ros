// #include <ros.h>
#include <Arduino.h>
#include <Wire.h>
#include "MS5837.h"
// #include <std_msgs/Float64.h>

MS5837 sensor;

// Baud rate for serial communication with Blue Robotics Bar30 High-Resolution 300m Depth/Pressure Sensor
#define BAUD_RATE 9600

// 1 publisher for depth sensor
//ros::NodeHandle_<ArduinoHardware,1,1,128,128> nh;
// ros::NodeHandle nh;
// std_msgs::Float64 depth;
// ros::Publisher pressure_pub = ros::Publisher("/offboard/pressure", &depth);

void setup(){

    // Serial1.begin(0); //weird hack
    delay(100);
    // Serial1.end(); // Close the serial port
    // Serial1.begin(9600);
    Serial.begin(9600);

    // nh.getHardware()->setBaud(BAUD_RATE);
    // nh.initNode();
    // nh.advertise(pressure_pub);

    Wire.begin();

    while (!sensor.init()) {
        // nh.logwarn("Pressure sensor not initialized. Will attempt every second until found.");
        delay(1000);
        break;
    }

    sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
}

void loop(){

    sensor.read();
    // depth = std_msgs::Float64();
    // depth.data = sensor.depth();
    
    // pressure_pub.publish(&depth);
    Serial.flush();
    // nh.spinOnce();

    //publish the depth over serial
    Serial.println(sensor.depth());
}