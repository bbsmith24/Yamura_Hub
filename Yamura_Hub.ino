//
// ESP-NOW data logger hub
// receive data from nodes, write to memory/sd card, etc
// upload file to PC for analysis
//
//#define PRINT_RECIEVED
//#define DEBUG_PRINT
//#define TEST_IO
#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SD.h"
#define TEST_DIGITAL 15
#define BUILTIN_LED 2

// all known peers - for sending time message
const uint8_t all_peers[6] {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

struct AccelLeaf
{
  char leafType;            //  1 byte  - type, in this case 'A' for accelerometer
  unsigned long timeStamp;  //  4 bytes - millis() value of sample
  int16_t values[6];        // 12 bytes - X,Y,Z acceleration values and i, j, k rotation values
} accelData;
//
struct IOLeaf
{
  char leafType;            // 1 byte  - type, in this case 'I' for IO - might need to make this 2 characters...
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
  int16_t a2dValues[4];     // 8 bytes, 2 per a2d channel
  int16_t digitalValue;     // 1 byte, 1 bit per digital channel
} ioData;
//
struct GPSLeaf
{
  char leafType;            //  1 byte  - type, in this case 'G' for GPS
  unsigned long timeStamp;  //  4 bytes - micros() value of sample
  char nmeaTime[15];        // 10 bytes of nmea time string in form hhmmss.sss
  char gpsLatitude[15];       //  9 bytes of nmea latitude in form ddmm.mmmm              
  char gpsLongitude[15];      // 10 bytes of nmea longitude in form dddmm.mmmm              
} gpsData;
//
struct HubTimeStamp
{
  char msgType;
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
};
union HubTimeStampPacket
{
  HubTimeStamp hubMsg;
  uint8_t timeStamp[5];
} hubTS;

int testState = LOW;
unsigned long lastTest = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(TEST_DIGITAL, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(TEST_DIGITAL, testState);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  #ifdef DEBUG_PRINT
  Serial.println("YamuraLog ESPNow Multi-node & Hub Test");
  // This is the mac address of this device
  Serial.print("Hub MAC: "); Serial.println(WiFi.macAddress());
  #endif
  // init EPS-NOW
  InitESPNow();
  // register for send callback to get the status of the sent packet
  esp_now_register_send_cb(OnDataSent);
  // register for receive callback to get the received packet
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  // for testing sync times with A2D leaf
  #ifdef TEST_IO
  if(micros() - lastTest > 1000000)
  {
    lastTest = micros();
    testState = testState == LOW ? HIGH : LOW;
    digitalWrite(TEST_DIGITAL, testState);
    digitalWrite(BUILTIN_LED, testState);
    Serial.print(lastTest);
    Serial.print(" Test digital ");
    Serial.println((testState == LOW ? "HIGH" : "LOW"));
  }
  #endif
}
//
// initialize ESP-NOW
//
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    #ifdef DEBUG_PRINT
    Serial.println("ESPNow Init Success");
    #endif
  }
  else
  {
    #ifdef DEBUG_PRINT
    Serial.println("ESPNow Init Failed");
    #endif
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
//
// callback when data is sent
//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if(status != ESP_NOW_SEND_SUCCESS)
  {
    #ifdef DEBUG_PRINT
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print("Last Packet Sent to: "); Serial.println(macStr);
    Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
    #endif
  }
}
//
// callback when data is received
//
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  #ifdef PRINT_RECIEVED
  Serial.print(micros());
  Serial.print("\t");
  for(int idx = 0; idx < 6; idx++)
  {
    Serial.print(mac_addr[idx], HEX);Serial.print(":");
  }
  Serial.print("\t");
  #endif
  switch(data[0])
  {
    // accelerometer leaf data
    case 'A':
      memcpy(&accelData, data, sizeof(accelData));
      #ifdef PRINT_RECIEVED
      Serial.print("Time\t"); Serial.print(accelData.timeStamp); Serial.print("\t");
      Serial.print("ACCEL\t"); 
      Serial.print(accelData.values[0]); Serial.print("\t");
      Serial.print(accelData.values[1]); Serial.print("\t");
      Serial.print(accelData.values[2]); Serial.print("\tGYRO\t");
      Serial.print(accelData.values[3]); Serial.print("\t");
      Serial.print(accelData.values[4]); Serial.print("\t");
      Serial.println(accelData.values[5]);
      #endif 
      break;
    // digital/analog IO leaf data
    case 'I':
      memcpy(&ioData, data, sizeof(ioData));
      #ifdef PRINT_RECIEVED
      Serial.print("Time\t"); Serial.print(ioData.timeStamp); Serial.print("\tA2D\t");
      for(int idx = 0; idx < 4; idx++)
      {
        Serial.print(ioData.a2dValues[idx]); Serial.print("\t");
      }
      Serial.print("\tdigital\t"); Serial.println(ioData.digitalValue, BIN);
      #endif
      break;
    // GPS leaf data
    case 'G':
      memcpy(&gpsData, data, sizeof(gpsData));
      #ifdef PRINT_RECIEVED
      Serial.print("Time\t"); Serial.print(gpsData.timeStamp); Serial.print("\tGPS\t");
      Serial.print("nmeaTime\t"); Serial.print(gpsData.nmeaTime); Serial.print("\t");
      Serial.print("long\t"); Serial.print(gpsData.gpsLatitude); Serial.print("\t");
      Serial.print("lat\t"); Serial.println(gpsData.gpsLongitude);
      #endif
      break;
    // time request message
    case 'T':
    {
      esp_now_peer_info *peer = new esp_now_peer_info();
      peer->peer_addr[0]= mac_addr[0];
      peer->peer_addr[1]= mac_addr[1];
      peer->peer_addr[2]= mac_addr[2];
      peer->peer_addr[3]= mac_addr[3];
      peer->peer_addr[4]= mac_addr[4];
      peer->peer_addr[5]= mac_addr[5];
      peer->channel = 1;
      peer->encrypt = false;
      peer->priv = NULL;
      esp_now_add_peer((const esp_now_peer_info_t*)peer);
      hubTS.hubMsg.msgType = 'T';
      hubTS.hubMsg.timeStamp = micros();


      #ifdef PRINT_RECIEVED
      Serial.print("Send timestamp to "); 
      for(int idx = 0; idx < 6; idx++)
      {
        Serial.print(":");
        Serial.print(mac_addr[idx], HEX);
      }      
      Serial.print(" (");
      Serial.print(sizeof(HubTimeStamp));
      Serial.print(" bytes)");
      Serial.println(hubTS.hubMsg.timeStamp);
      #endif
      uint8_t result = esp_now_send(mac_addr, &hubTS.timeStamp[0], sizeof(HubTimeStamp));
      break;
    }
    default:
      #ifdef DEBUG_PRINT
      Serial.print("Unknown data type "); Serial.println((char)data[0]);
      #endif
      break;
  }
}
