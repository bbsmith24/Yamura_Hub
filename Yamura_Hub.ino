//
// Yamura Logger hub test
// 
// receive data from leafs, write to memory/sd card, etc
// upload file to PC for analysis
//
#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "DataStructures.h"

#define PRINT_RECIEVED
#define PRINT_DEBUG
//#define TEST_IO
#define BUILTIN_LED 2
#define LOG_BUTTON    4
#define TEST_DIGITAL 15
#define TIMESTAMP_BROADCAST 15000000
enum LogState
{
  OFF,
  ON,
  UNDEFINED
};

// data packets
TimeStampPacket timeStampPacket;
AccelPacket accelPacket;
IOPacket    ioPacket;
GPSPacket   gpsPacket;
// ESP-NOW
#define MAX_PEERS 20 // ESP-NOW peer limit
int peerCnt = 0;
esp_now_peer_info_t peers[MAX_PEERS] = {};
//
int testState = LOW;
unsigned long lastLogChange = 0;
unsigned long lastTime = 0;
//
unsigned long lastBroadcastTime = 0;
unsigned long rcvTime = 0;
// logging
volatile LogState isLogging = LogState::UNDEFINED;
bool logBtnPress;
unsigned long logBtnDebounceStart = 0;
int nextLogIdx = 0;
char nextLogName[256];
File currentLogFile;
char msgOut[256];
// max data packet size os 250 bytes
uint8_t dataBytes[250];
int dataBytesCnt = 0;
TaskHandle_t Task1;
uint8_t lastMac[6];

void setup()
{
  Serial.begin(115200);
  // IO pin setup
  pinMode(TEST_DIGITAL, OUTPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(LOG_BUTTON, INPUT_PULLUP);
  digitalWrite(TEST_DIGITAL, testState);
  // ESP-NOW setup
  for(int peerIdx = 0; peerIdx < MAX_PEERS; peerIdx++)
  {
    for(int macIdx = 0; macIdx < 6; macIdx++)
    {
      peers[peerIdx].peer_addr[macIdx] = 0;
    }
  }
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.println("YamuraLog ESPNow Multi-node & Hub Test");
  // This is the mac address of this device
  Serial.println("HUB MAC: "); Serial.println(WiFi.macAddress());
  while(!InitSD())
  {
    #ifdef PRINT_DEBUG
    Serial.println("No SD card found. Retry...");
    #endif
    delay(1000);
  }
  // get log file name
  FindNextLogfile();
  // init EPS-NOW
  InitESPNow();
  // register for send and receive callback functions
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  // initialize uSD card
  // not initially logging
  isLogging = LogState::UNDEFINED;
  logBtnPress = false;
  logBtnDebounceStart = 0;
  lastBroadcastTime = micros();

  xTaskCreatePinnedToCore(CheckLogButton,
                          "CheckLogButton",
                          10000,
                          NULL,
                          1,
                          &Task1, 
                          0);
  Serial.println("Running");
}

void loop() 
{
  // if not logging, send timestamps to peers periodically
  if((isLogging!= LogState::ON) && (micros() - lastBroadcastTime > TIMESTAMP_BROADCAST))
  {
    BroadcastTimestamp();
    lastBroadcastTime = micros();
  }
  if(dataBytesCnt > 0)
  {
    if(isLogging == LogState::ON)
    {
      if(dataBytes[0] == 'I')
      {
        memcpy(&ioPacket, dataBytes, sizeof(ioPacket));
        dataBytesCnt = 0;
        //AddPeer(lastMac);
        sprintf(msgOut, "I %lu\t%d\t%d\t%d\t%d\t%02X\n", ioPacket.packet.timeStamp, 
                                                         ioPacket.packet.a2dValues[0], ioPacket.packet.a2dValues[1], ioPacket.packet.a2dValues[2], ioPacket.packet.a2dValues[3], 
                                                         ioPacket.packet.digitalValue);
        Serial.print(msgOut);
      }
      else if(dataBytes[0] == 'A')
      {
        memcpy(&accelPacket, dataBytes, sizeof(accelPacket));
        dataBytesCnt = 0;
        sprintf(msgOut, "A %lu\t%d\t%d\t%d\t%d\t%d\t%d\n", accelPacket.packet.timeStamp, 
                                                         accelPacket.packet.values[0], accelPacket.packet.values[1], accelPacket.packet.values[2],
                                                         accelPacket.packet.values[3], accelPacket.packet.values[4], accelPacket.packet.values[5]);
        Serial.print(msgOut);
      }
      else if(dataBytes[0] == 'G')
      {
        memcpy(&gpsPacket, dataBytes, sizeof(gpsPacket));
        dataBytesCnt = 0;
        sprintf(msgOut, "G %lu\t%s\t%s\t%s\n", gpsPacket.packet.timeStamp, 
                                                           gpsPacket.packet.nmeaTime, gpsPacket.packet.gpsLatitude, gpsPacket.packet.gpsLongitude);
        Serial.print(msgOut);
      }
      // received timestamp request or heartbeat while logging - update state
      else if((dataBytes[0] == 'T') || (dataBytes[0] == 'H'))
      {
        BroadcastTimestamp();
        SendLoggingState(lastMac);
      }
    }
    else
    {
      if(dataBytes[0] == 'T')
      {
        snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tTIMESTAMP REQ\n",
             micros(), 
             lastMac[0], lastMac[1], lastMac[2], lastMac[3], lastMac[4], lastMac[5]);
        Serial.print(msgOut);
      }
      else if(dataBytes[0] == 'H')
      {
        snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tHEARTBEAT RCV\n",
             micros(), 
             lastMac[0], lastMac[1], lastMac[2], lastMac[3], lastMac[4], lastMac[5]);
        Serial.print(msgOut);
      }
      // received data while idle - update state
      else if((dataBytes[0] == 'A') || (dataBytes[0] == 'I') || (dataBytes[0] == 'G'))
      {
        SendLoggingState(lastMac);
      }
      
      dataBytesCnt = 0;
      AddPeer(lastMac);
    }
  }
  // check log button state
  //CheckLogButton();
}
//
// check for log button press/release to change state
//
void CheckLogButton(void * pvParameters )
{
  unsigned long curTime = millis();
  while(true)
  {
    delay(100);
    //if(millis() - curTime < 100)
    //{
    //  continue;
    //}
    curTime = millis();
    // check for log button press with 1 sec debounce
    logBtnDebounceStart = 0;
    while(digitalRead(LOG_BUTTON) == LOW)
    {
    }
    if(millis() - curTime >= 1000)
    {
        #ifdef PRINT_DEBUG
        Serial.print("CheckLogButton on core ");
        Serial.println(xPortGetCoreID());
        Serial.print(" Button release, send logging change to ");
        Serial.println((isLogging == LogState::ON ? "LogState::OFF" : "LogState::ON"));
        #endif
        SendLoggingChange((isLogging == LogState::ON ? LogState::OFF : LogState::ON));
    }
  }
}
//
//
//
bool InitSD()
{
  if(!SD.begin())
  {
      #ifdef PRINT_DEBUG
      Serial.println("Card Mount Failed");
	  #endif
      return false;
  }
  uint8_t cardType = SD.cardType();
  switch(cardType)
  {
    case CARD_NONE:
      #ifdef PRINT_DEBUG
      Serial.println("No SD card attached");
	  #endif
      return false;
      break;
    case CARD_MMC:
      #ifdef PRINT_DEBUG
      Serial.print("MMC ");
	  #endif
      break;
    case CARD_SD:
      #ifdef PRINT_DEBUG
      Serial.print("SDSC ");
	  #endif
      break;
    case CARD_SDHC:
      #ifdef PRINT_DEBUG
      Serial.print("SDHC ");
	  #endif
      break;
    default:
      #ifdef PRINT_DEBUG
      Serial.print("Unknown card type ");
	  #endif
	  return false;
      break;
  }
  #ifdef PRINT_DEBUG
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card Size: %lulMB\n", cardSize);
  #endif
  return true;
}
//
// initialize ESP-NOW
//
void InitESPNow()
{
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK)
  {
    #ifdef PRINT_DEBUG
    Serial.println("ESPNow Init Success");
    #endif
  }
  else
  {
    #ifdef PRINT_DEBUG
    Serial.println("ESPNow Init Failed - restart");
    #endif
    ESP.restart();
  }
}
//
// handle change in logging state
//
void SendLoggingChange(LogState newState)
{
  if(newState == isLogging)
  {
    return;
  }
  isLogging = newState;
  char msgType = isLogging == LogState::ON ? 'B' : 'E';
  // logging started
  if(isLogging == LogState::ON)
  {
    currentLogFile = SD.open(nextLogName);
    #ifdef PRINT_DEBUG
    Serial.print(micros());
    Serial.println(" START logging");
    #endif
  }
  // logging ended
  else
  {
    // close current file
    currentLogFile.close();
    // find next file
    #ifdef PRINT_DEBUG
    Serial.print(micros());
    Serial.println(" END logging");
    #endif
    FindNextLogfile();    
  }
  uint8_t result = esp_now_send(0, (const uint8_t*)&msgType, sizeof(msgType));
  #ifdef PRINT_DEBUG
  for(int peerIdx = 0; peerIdx < peerCnt; peerIdx++)
  {
    Serial.print("Send "); 
    Serial.print(isLogging == LogState::ON ? "START LOGGING to " : "END LOGGING to ");
    snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tLogging change\t%010ld\tResult\t%d\n",
             micros(), 
             peers[peerIdx].peer_addr[0], peers[peerIdx].peer_addr[1], peers[peerIdx].peer_addr[2], peers[peerIdx].peer_addr[3], peers[peerIdx].peer_addr[4], peers[peerIdx].peer_addr[5],
             timeStampPacket.packet.timeStamp,
             result);
    Serial.print(msgOut);
  }
  #endif
}
//
// send current logging state to out-of-phase leaf
// (logging when hub is idle, idle when hub is logging)
//
void SendLoggingState(uint8_t *macAddr)
{
  char msgType = isLogging == LogState::ON ? 'B' : 'E';
  uint8_t result = esp_now_send(macAddr, (const uint8_t*)&msgType, sizeof(msgType));
  #ifdef PRINT_DEBUG
  Serial.print("Send current hub logging state "); 
  Serial.print(isLogging == LogState::ON ? "LOGGING to " : "IDLE to ");
  snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\n",
           micros(), 
           macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
  Serial.print(msgOut);
  #endif
}
//
// callback when data is sent
//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  if(status != ESP_NOW_SEND_SUCCESS)
  {
    #ifdef PRINT_DEBUG
    snprintf(msgOut, sizeof(msgOut), "Last Packet Sent to\t%02X:%02X:%02X:%02X:%02X:%02X\tstatus\t%s\n", 
	         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
			 (status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail"));
    Serial.print(msgOut);
    #endif
  }
}
//
// callback when data is received
// just receive data and update byte count here, acting on type of data sent is handled elsewhere
// to keep this funcion fast
//
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(&lastMac, mac_addr, 6);
  memcpy(&dataBytes, data, data_len);
  dataBytesCnt = data_len;
}
//
//
//
void AddPeer(const uint8_t* mac_addr)
{
  bool peerFound = false;
  int peerIdx = 0;
  while((peerIdx < peerCnt) && (peerFound == false))
  {
    peerFound = true;
    for(int macIdx = 0; macIdx < 6; macIdx++)
    {
      if(peers[peerIdx].peer_addr[macIdx] != mac_addr[macIdx])
      {
        peerFound = false;
      }
    }
    peerIdx++;
  }
  if(!peerFound)
  {
    for(int macIdx = 0; macIdx < 6; macIdx++)
    {
      peers[peerCnt].peer_addr[macIdx]= mac_addr[macIdx];
    }
    peers[peerCnt].channel = 1;
    peers[peerCnt].encrypt = false;
    peers[peerCnt].priv = NULL;
    uint8_t result = esp_now_add_peer(&peers[peerCnt]);
    peerCnt++;
    #ifdef PRINT_DEBUG
    if(result == 0)
    {
      snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tadded as peer %d\n", 
               micros(), 
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
               (peerCnt));
    }
    else
    {
      snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tNOT added as peer ERROR %d\n", 
               rcvTime, 
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], 
               result);
    }
    Serial.print(msgOut);
    #endif
  }
  else
  {
     snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\talready a peer\n", 
               rcvTime, 
               mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
     
  }
}
//
//
//
void BroadcastTimestamp()
{
  // setup timestamp
  uint8_t result;
  timeStampPacket.packet.msgType[0] = 'T';
  timeStampPacket.packet.timeStamp = micros();
  // send to all known peers
  result = esp_now_send(0, &timeStampPacket.dataBytes[0], sizeof(timeStampPacket));
  #ifdef PRINT_DEBUG
  for(int peerIdx = 0; peerIdx < peerCnt; peerIdx++)
  {
    snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tBroadcast timestamp\t%lu\n",
             timeStampPacket.packet.timeStamp, 
             peers[peerIdx].peer_addr[0], peers[peerIdx].peer_addr[1], peers[peerIdx].peer_addr[2], peers[peerIdx].peer_addr[3], peers[peerIdx].peer_addr[4], peers[peerIdx].peer_addr[5],
             timeStampPacket.packet.timeStamp);
    Serial.print(msgOut);
  }
  #endif
}
//
// get next sequential log file name
//
void FindNextLogfile()
{
  Serial.println("Finding next LOG file to create...");
  int levels = 0;
  File root = SD.open("/");
  if((!root) || (!root.isDirectory()))
  {
    #ifdef PRINT_DEBUG
    Serial.println("Failed to open requested directory");
	  #endif
    return;
  }

  File file = root.openNextFile();
  nextLogIdx = 0;
  while(file)
  {
    if(!file.isDirectory()) // not a directory, therefore a file
    {
      String fileNameStr = String(file.name());
      if(fileNameStr.startsWith("/Log_"))
      {
        nextLogIdx++;
      }
    }
    file = root.openNextFile();
  }
  #ifdef PRINT_DEBUG
  sprintf(nextLogName, "/Log_%04d%s", nextLogIdx,".ylg");
  Serial.print("Next log file : ");
  Serial.println(nextLogName);
  #endif
}
//
// write to file
//
void writeFile(fs::FS &fs, const char * path, const char * message, int messageLen)
{
  File file = fs.open(path, FILE_APPEND);
  //file.write(message, messageLen);
  //file.print(message);
  file.close();
}
void printFile(fs::FS &fs, const char * path, const char * message)
{
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}
//
// append to file
//
void appendFile(fs::FS &fs, const char * path, const char * message)
{
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}
