#include <wifi_mouse.h>

WiFiUDP udp;
boolean connected = false;


//struct for udp packets
struct mouse{
  float heading_req;
  float velocity_req;
  float heading_meas;
  float velocity_meas;
  int mouse_state;
  int move_req;
  float battery_voltage;
} mouse;

struct brain{
  float heading_req;
  float velocity_req;
  int move_req;
} brain;

void connectToWiFi(const char * ssid, const char * pwd){
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);
  
  //Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          connected = false;
          break;
      default: break;
    }
}


void send_udp(struct mouse*){
 //Send a packet
    udp.beginPacket(udpAddress,udpPort);
    udp.write((uint8_t*)&mouse, sizeof(mouse));
    udp.endPacket();
}

void rx_udp(struct brain*){
  //read recieved packet
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  udp.read((uint8_t*)&brain, sizeof(brain));
}

