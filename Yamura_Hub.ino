//
// ESP-NOW data logger hub
// receive data from nodes, write to memory/sd card, etc
// upload file to PC for analysis
//
#include <esp_now.h>
#include <WiFi.h>
#include "BluetoothSerial.h"
#include "FS.h"
#include "SD.h"

struct AccelLeaf
{
  char leafType;            // 1 byte  - type, in this case 'A' for accelerometer
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
  int16_t values[3];        // 6 bytes - X,Y,Z acceleration values
} accelData;
//
struct IOLeaf
{
  char leafType;            // 1 byte  - type, in this case 'I' for IO - might need to make this 2 characters...
  unsigned long timeStamp;  // 4 bytes - millis() value of sample
  int16_t a2dValues[4];     // 8 bytes, 2 per a2d channel
  int16_t digitalValue;     // 1 byte, 1 bit per digital channel
} ioData;

BluetoothSerial SerialBT;
String btName = "XGPS160-45E134";
char *btPin = "1234"; //<- standard pin would be provided by default
bool btConnected;

void setup()
{
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  Serial.println("YamuraLog ESPNow Multi-node & Hub Test");
  // This is the mac address of this device
  Serial.print("Hub MAC: "); Serial.println(WiFi.macAddress());
  // init EPS-NOW
  InitESPNow();
  // register for send callback to get the status of the sent packet
  esp_now_register_send_cb(OnDataSent);
  // register for receive callback to get the received packet
  esp_now_register_recv_cb(OnDataRecv);

 // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
  // to resolve name to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices bluetooth address and device names
  Serial.print("Attempt to connect to "); Serial.println(btName);

  SerialBT.begin("ESP32test", true); 
  btConnected = SerialBT.connect(btName);
  
  if(btConnected) 
  {
    Serial.println("Connected Succesfully!");
  }
  else 
  {
    while(!SerialBT.connected(10000)) 
    {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) 
  {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
}

void loop()
{
  if(SerialBT.available() > 0)
  {
    String gpsRecieved = "";
    String outStr = "";
    byte inByte;
    // read available byte(s)
    while (SerialBT.available()) 
    {
      //CheckSamples();
      inByte = SerialBT.read();
      if(inByte == 10)
      {
        break;
      }
      gpsRecieved.concat((char)inByte);
    }
    if(gpsRecieved.startsWith("$GPRMC"))
    {
      Serial.print("Recv from: "); Serial.print(btName); Serial.print(": Time ");
      Serial.print(millis());
      Serial.print(" ");
      Serial.println(gpsRecieved);
      //outStr.concat(millis());
      //outStr.concat(" ");
      //Serial.print(outStr);
      //LogPrint(gpsRecieved);
    }
  }
}
//
//
//
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    Serial.println("ESPNow Init Success");
  }
  else
  {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}
// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
//
//
//
union DataRecieved
{
  unsigned long timeStamp;
  char  timeStampBytes[4];
} dataRecv;

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  Serial.print("Recv from: ");
  for(int idx = 0; idx < 6; idx++)
  {
    Serial.print(mac_addr[idx], HEX);Serial.print(":");
  }
  Serial.print(" ");
  switch(data[0])
  {
    case 'A':
      memcpy(&accelData, data, sizeof(accelData));
      Serial.print("Time "); Serial.print(accelData.timeStamp); Serial.print(" ");
      Serial.print("Accel "); Serial.print(accelData.values[0]); Serial.print(" ");
      Serial.print(accelData.values[1]); Serial.print(" ");
      Serial.println(accelData.values[2]);
      break;
    case 'I':
      memcpy(&ioData, data, sizeof(ioData));
      Serial.print("Time "); Serial.print(ioData.timeStamp); Serial.print(" A2D ");
      for(int idx = 0; idx < 4; idx++)
      {
        Serial.print(ioData.a2dValues[0]); Serial.print(" ");
      }
      Serial.print(" digital "); Serial.println(ioData.digitalValue, BIN);
      break;
    
    default:
      Serial.print("Unknown data type "); Serial.println((char)data[0]);
      break;
  }
}
