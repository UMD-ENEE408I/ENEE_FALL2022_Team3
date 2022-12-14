#include <Arduino.h>
#include "DefaultLoopTrajectory.cpp"
#include "wifi_mouse.cpp"
#include <Adafruit_MCP3008.h>
#include <iostream>

const char* ssid1 = "Test";
const char* password1 =  "OPkl!234";


struct brain brain1;
struct mouse mouse1;

void setup() {
  //setWifi(ssid,password);  //wifitest setup
  setDefault_Trajectory(); //TrajectoryTracking setup
  // delete old config
  WiFi.disconnect(true);
  //Connect to the WiFi network
  connectToWiFi(ssid1, password1);
}

void loop() {
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  int start = 0;
  brain1.move_req = 1;
  int last_mode = 1;

  //stays here after initial
  while(true) {
  //rx message from server
  rx_udp(&brain1);


  //default circle is 0
  if (brain1.move_req != last_mode)  {
    start = 0;
  }

  float* speedangle;//2 variable pointer[0] is v, [1] is w
  speedangle = defaultLoop(enc1, enc2, start, brain1.move_req);  
  Serial.print("\tvelocity: "); Serial.print(speedangle[0]); Serial.print("\t"); Serial.print("angle: ");Serial.print(speedangle[1]); 
  Serial.print("\n");
  start = 1;
  mouse1.heading_meas = speedangle[1];
  mouse1.velocity_meas = speedangle[0];
  last_mode = brain1.move_req;
  
  //send to server
  send_udp(&mouse1);
  }
  
}