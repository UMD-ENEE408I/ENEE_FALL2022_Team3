#include <Arduino.h>
#include <TrajectoryTracking.h>
#include <wifitest.h>
#include <Adafruit_MCP3008.h>
const char* ssid = "ssid";
const char* password =  "password";

void setup() {
  setWifi(ssid,password);  //runs wifitest setup
  setDefault_Trajectory(); //runs the TrajectoryTracking setup
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
}