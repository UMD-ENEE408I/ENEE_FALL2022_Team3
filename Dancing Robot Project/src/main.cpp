#include <Arduino.h>
#include "DefaultLoopTrajectory.cpp"
#include <wifi_mouse.h>
#include <Adafruit_MCP3008.h>
#include <iostream>
const char* ssid = "Test";
const char* password =  "OPkl!234";
const char * udpAddress = "192.168.26.101";
const int udpPort = 3333;

struct brain brain;
struct mouse mouse;

void setup() {
  //setWifi(ssid,password);  //wifitest setup
  setDefault_Trajectory(); //TrajectoryTracking setup
  // delete old config
  WiFi.disconnect(true);
  //Connect to the WiFi network
  connectToWiFi(ssid, password);
}

void loop() {
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  int start = 0;
  while(true) {
  //rx message from server
  rx_udp(&brain);

  float* speedangle;//2 variable pointer[0] is v, [1] is w
  speedangle = defaultLoop(enc1, enc2, start);  //TrajectoryTracking loop, should later be modified to change loop shape
  Serial.print("\tvelocity: "); Serial.print(speedangle[0]); Serial.print("\t"); Serial.print("angle: ");Serial.print(speedangle[1]); 
  Serial.print("\n");
  start = 1;

  //send to server
  send_udp(&mouse);
  }
  
}