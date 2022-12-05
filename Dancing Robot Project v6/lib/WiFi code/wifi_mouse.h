#include <WiFi.h>
#include <WiFiMulti.h>
#include <Arduino.h>
#include <Adafruit_MCP3008.h>
#include <WiFiUdp.h>

const char* ssid;
const char* password;
const char* udpAddress = "192.168.83.101";
const int udpPort = 3333;
WiFiUDP udp;

//struct for udp packets
struct mouse{
  
  float x;
  float y;
  float heading;
  int move;
  } mouse;

struct brain{
  float meas_x;
  float meas_y;
  float heading_meas;
  int move_req;
} brain;

void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);
void send_udp(struct mouse &mouse);
void rx_udp(struct brain &brain);
