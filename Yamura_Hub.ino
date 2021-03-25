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

//#define PRINT_CORE
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
  #ifdef PRINT_CORE
  Serial.print("Setup core ");
  Serial.println(xPortGetCoreID());
  #endif
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
  #ifdef PRINT_DEBUG
  Serial.println();
  Serial.println("YamuraLog ESPNow Multi-node & Hub Test");
  // This is the mac address of this device
  Serial.println("HUB MAC: "); Serial.println(WiFi.macAddress());
  #endif
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
}

void loop() 
{
  // if not logging, send timestamps to peers periodically
  if((isLogging!= LogState::ON) && (micros() - lastBroadcastTime > TIMESTAMP_BROADCAST))
  {
    #ifdef PRINT_CORE
    Serial.print("loop core ");
    Serial.println(xPortGetCoreID());
    #endif  
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
    while((digitalRead(LOG_BUTTON) == LOW)/* && ((millis() - curTime) < 1000)*/)
    {
      //if(logBtnDebounceStart == 0)
      //{
      //  logBtnDebounceStart = millis();
      //  #ifdef PRINT_CORE
      //  #endif
      //}
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
  #ifdef PRINT_CORE
  Serial.print("InitSD core ");
  Serial.println(xPortGetCoreID());
  #endif
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
  #ifdef PRINT_CORE
  Serial.print("InitESPNow core ");
  Serial.println(xPortGetCoreID());
  #endif  
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
  #ifdef PRINT_CORE
  Serial.print("SendLoggingChange core ");
  Serial.println(xPortGetCoreID());
  #endif
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
    Serial.print("Send Logging "); 
    Serial.print(isLogging == LogState::ON ? "START to " : "END to ");
    snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tLogging change\t%010ld\tResult\t%d\n",
             rcvTime, 
             peers[peerIdx].peer_addr[0], peers[peerIdx].peer_addr[1], peers[peerIdx].peer_addr[2], peers[peerIdx].peer_addr[3], peers[peerIdx].peer_addr[4], peers[peerIdx].peer_addr[5],
             timeStampPacket.packet.timeStamp,
             result);
    Serial.print(msgOut);
  }
  #endif
}
//
// callback when data is sent
//
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  #ifdef PRINT_CORE
  Serial.print("OnDataSent core ");
  Serial.println(xPortGetCoreID());
  #endif
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
//
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  memcpy(&lastMac, mac_addr, 6);
  memcpy(&dataBytes, data, data_len);
  dataBytesCnt = data_len;
  /*
  #ifdef PRINT_CORE
  Serial.print("OnDataRecv core ");
  Serial.println(xPortGetCoreID());
  #endif
  rcvTime = micros();
  switch(data[0])
  {
    // time request message
    case 'T':
    {
      AddPeer(mac_addr);
      timeStampPacket.packet.msgType[0] = 'T';
      timeStampPacket.packet.timeStamp = rcvTime;
      uint8_t result = esp_now_send(mac_addr, &timeStampPacket.dataBytes[0], sizeof(timeStampPacket));
      #ifdef PRINT_DEBUG
      snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tTimestamp request\n", 
               rcvTime, 
			         mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      Serial.print(msgOut);
      #endif
      break;
    }
    // leaf heartbeat message
    case 'H':
    {
      AddPeer(mac_addr);
      // shouldn't be getting heartbeats while logging - send start
      //if(isLogging == LogState::ON)
      //{
      //  #ifdef PRINT_DEBUG
      //  snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tHeartbeat received while logging LogState::OFF - send logging LogState::ON",
      //           rcvTime, 
      //           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      //  Serial.println(msgOut);
      //  #endif
      //  SendLoggingChange(LogState::ON);
      //  break;
      //}
      
      memcpy(&timeStampPacket, data, sizeof(timeStampPacket));
      #ifdef PRINT_DEBUG
      snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tHeartbeat: Hub Time\t%010lu\tLeaf Time\t%010lu\tDiff Time\t%lu\n",
               rcvTime, 
			   mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
               rcvTime,
               timeStampPacket.packet.timeStamp,
               (int)(rcvTime - timeStampPacket.packet.timeStamp));
      Serial.print(msgOut);
      #endif
      break;
    }
    // received from digital/analog input leaf
    case 'I':
    {
      // shouldn't be sending data while not logging (off or undefined) - send stop
      if((isLogging == LogState::OFF) || (isLogging == LogState::UNDEFINED))
      {
        #ifdef PRINT_DEBUG
        snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tData received while logging off - send logging LogState::OFF",
                 rcvTime, 
                 mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        Serial.println(msgOut);
        #endif
        AddPeer(mac_addr);
        SendLoggingChange(LogState::OFF);
        break;
      }
      memcpy(&ioPacket, data, sizeof(ioPacket));
      sprintf(msgOut, "I %lu\t%d\t%d\t%d\t%d\t%02X\n", ioPacket.packet.timeStamp, 
                                                     ioPacket.packet.a2dValues[0], ioPacket.packet.a2dValues[1], ioPacket.packet.a2dValues[2], ioPacket.packet.a2dValues[3], 
                                                     ioPacket.packet.digitalValue);
      printFile(SD, nextLogName, msgOut);
      #ifdef PRINT_RECIEVED
      //Serial.print(rcvTime);
      //Serial.print("\t");
      //Serial.print(mac_addr[0], HEX);
      //for(int idx = 1; idx < 6; idx++)
      //{
      //  Serial.print(":");
      //  Serial.print(mac_addr[idx], HEX);
      //}
      //Serial.print("\t");Serial.print(msgOut);
      #endif
      break;
    }
    case 'A':
    {
      break;      
    }
    case 'G':
    {
      break;      
    }
    default:
    {
      AddPeer(mac_addr);
      //if(!isLogging)
      //{
      //  SendLoggingChange();
      //  break;
      //}
      #ifdef PRINT_DEBUG
      Serial.print("Unknown data type "); Serial.println((char)data[0]);
      #endif
      break;
    }
  }
  */
}
//
//
//
void AddPeer(const uint8_t* mac_addr)
{
  #ifdef PRINT_CORE
  Serial.print("AddPeer core ");
  Serial.println(xPortGetCoreID());
  #endif
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
  #ifdef PRINT_CORE
  Serial.print("FindNextLogfile core ");
  Serial.println(xPortGetCoreID());
  #endif
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
  #ifdef PRINT_CORE
  Serial.print("writeFile core ");
  Serial.println(xPortGetCoreID());
  #endif
  File file = fs.open(path, FILE_APPEND);
  //file.write(message, messageLen);
  //file.print(message);
  file.close();
}
void printFile(fs::FS &fs, const char * path, const char * message)
{
  #ifdef PRINT_CORE
  Serial.print("printFile core ");
  Serial.println(xPortGetCoreID());
  #endif
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}
//
// append to file
//
void appendFile(fs::FS &fs, const char * path, const char * message)
{
  #ifdef PRINT_CORE
  Serial.print("appendFile core ");
  Serial.println(xPortGetCoreID());
  #endif
  File file = fs.open(path, FILE_APPEND);
  file.print(message);
  file.close();
}
