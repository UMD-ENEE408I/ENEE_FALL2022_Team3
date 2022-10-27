#include <wifi_mouse.h>

boolean connected = false;


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


void send_udp(struct mouse &mouse){
 //Send a packet
 char* mouse_string = reinterpret_cast<char*>(&mouse);
    udp.beginPacket(udpAddress,udpPort);
    udp.write((uint8_t*)&mouse, sizeof(mouse));
    udp.endPacket();
}

void rx_udp(struct brain &brain){
  //read recieved packet
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  udp.read((uint8_t*)&brain, sizeof(brain));
}

