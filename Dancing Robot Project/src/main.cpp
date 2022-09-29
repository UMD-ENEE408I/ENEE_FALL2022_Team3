#include <Arduino.h>
#include <TrajectoryTracking.h>
#include <wifitest.h>
#include <Adafruit_MCP3008.h>
const char* ssid = "ssid";
const char* password =  "password";

void setup() {
  setWifi(ssid,password);  //wifitest setup
  setDefault_Trajectory(); //TrajectoryTracking setup
}

void loop() {
  defaultLoop();  //TrajectoryTracking loop, should later be modified to change loop shape + provided device info for jetson
}