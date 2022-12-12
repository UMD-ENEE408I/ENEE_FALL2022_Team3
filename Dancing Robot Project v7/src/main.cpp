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
  mouse1.heading = 0.0;
  mouse1.x = 0.0;
  mouse1.y = 0.0;
  mouse1.move = 0.0;

  send_udp(mouse1);
}

void loop() {
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);
  int start = 0; 
  //brain1.move_req = 0;
  int last_mode = 0;
  float x = 0.0;  //purely temporary
  float y = 0.0;  //for testing x/y position additions
  brain1.meas_x == 0.0;
  float last_x = 999.0;
  float last_y = 999.0;
  float newx = 1.0;
  float newy = 1.0;
  float intrx = 0.0; //the x and y from the code passed by reference, "internal"
  float intry = 0.0;
  int mode = 1;
  int modecount = 0;
  static float bias_omega;
  est_imu_bias(bias_omega, 500);// Could be expanded for more quantities

  while(brain1.meas_x == 0.0){
    send_udp(mouse1);
    rx_udp(brain1);

  }

  //update internal x,y coords
  intrx = brain1.meas_x;
  intry = brain1.meas_y;
  
  //actual loop for dance routine
  while(true) {
  //rx message from server
  rx_udp(brain1);

  //default circle is 0
  if (brain1.move_req != last_mode)  {
    start = 0;
  }

  //handles instance where brain1 did not update x or y
  if (last_x = brain1.meas_x) {
    x = intrx;
  }
  if (last_y = brain1.meas_y) {
    y = intry;
  }

  /* for testing during mode developement only
    modecount++;
  if (modecount % 500 == 0) {
    if (mode == 1)  {
      mode = 8;
    } else {
      mode = 1;
    }
    start = 0;
    newx = x;
    newy = y;
    Serial.println();
    Serial.print(start);  //resetting the start command causes a huge pause in the program where things go bad
    Serial.println();
  }
  */


  float* speedangle;//2 variable pointer[0] is v, [1] is w

  //0 = stop, 1 is big circle, 2 backwards circle, 4 zig-zag, 
  speedangle = defaultLoop(enc1, enc2, start, brain1.move_req, brain1.meas_x, brain1.meas_y, newx, newy, intrx, intry, bias_omega);  //brain1.move_req
  start = 1;
  last_x = brain1.meas_x;
  last_y = brain1.meas_y;
  last_mode = brain1.move_req;
  mouse1.heading = brain1.heading_meas;
  mouse1.x = brain1.meas_x;
  mouse1.y = brain1.meas_y;
  mouse1.move = brain1.move_req;

  send_udp(mouse1);
  
  }
  
}