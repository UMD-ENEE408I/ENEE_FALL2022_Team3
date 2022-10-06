#include <Arduino.h>
#include "DefaultLoopTrajectory.cpp"
#include <wifitest.h>
#include <Adafruit_MCP3008.h>
#include <iostream>
const char* ssid = "ssid";
const char* password =  "password";

void setup() {
  //setWifi(ssid,password);  //wifitest setup
  setDefault_Trajectory(); //TrajectoryTracking setup
}

void loop() {
  float* speedangle;//2 variable pointer[0] is v, [1] is w
  speedangle = defaultLoop();  //TrajectoryTracking loop, should later be modified to change loop shape + provided info to jetson
  Serial.print("velocity: "); Serial.print(speedangle[0]); Serial.print("\t"); Serial.print("angle: ");Serial.print(speedangle[1]); 
  Serial.print("\n");
}