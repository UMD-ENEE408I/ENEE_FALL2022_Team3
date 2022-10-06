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
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  while(true) {
  float* speedangle;//2 variable pointer[0] is v, [1] is w
  speedangle = defaultLoop(enc1, enc2);  //TrajectoryTracking loop, should later be modified to change loop shape + provided info to jetson
  Serial.print("\tvelocity: "); Serial.print(speedangle[0]); Serial.print("\t"); Serial.print("angle: ");Serial.print(speedangle[1]); 
  Serial.print("\n");
  }
  
}