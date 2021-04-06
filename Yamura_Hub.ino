//
// Yamura Logger hub test
// 
// receive data from leafs, write to memory/sd card, etc
// upload file to PC for analysis
//
#include <esp_now.h>
#include <WiFi.h>
#include "FS.h"
#include <LITTLEFS.h>
#include "SPI.h"
#include "SdFat.h"
#include "sdios.h"
#include "DataStructures.h"
// SDFAT library defines
// SD_FAT_TYPE for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32
// 2 for exFAT
// 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
//
// Set DISABLE_CHIP_SELECT to disable a second SPI device.
// For example, with the Ethernet shield, set DISABLE_CHIP_SELECT
// to 10 to disable the Ethernet controller.
const int8_t DISABLE_CHIP_SELECT = -1;
//
// Test with reduced SPI speed for breadboards.  SD_SCK_MHZ(4) will select
// the highest speed supported by the board that is not over 4 MHz.
// Change SPI_SPEED to SD_SCK_MHZ(50) for best performance.
#define SPI_SPEED SD_SCK_MHZ(4)
#if SD_FAT_TYPE == 0
SdFat sd;
File logFileSD;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 logFileSD;
File32 currentLogFile;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile logFileSD;
ExFile currentLogFile;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile logFileSD;
//FsFile currentLogFile;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
// SD card chip select
// connections ESP32 WROOM
// GPIO23 (MOSI) to card SI
// GPIO19 (MISO) to card SO
// GPIO18 (SCK)  to card SCK
// GPIO5  (SS)   to card CS
// define chip select
//int chipSelect = 5; // HiLetgo Node32S
int chipSelect = 33;  // Sparkfun ESP32 Thing Plus

#define FORMAT_LITTLEFS_IF_FAILED true
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
IMUPacket imuPacket;
CombinedIOPacket    combinedIOPacket;
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
char nextLogName[13];
//BufferedPrint<File, 64> fileBuffer;
//BufferedPrint<FsFile, 64> fileBuffer;
// temp file in flash for this session
File flashFile;

char msgOut[256];
// max data packet size os 250 bytes
uint8_t dataBytes[250];

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
  peerCnt = 0;
  WiFi.mode(WIFI_STA);
  Serial.println();
  // This is the mac address of this device
  Serial.print("YamuraLog Hub ");Serial.println(WiFi.macAddress());
  while(!InitializeSD())
  {
    #ifdef PRINT_DEBUG
    Serial.println("No SD card or SD error. Retry...");
    #endif
    delay(1000);
  }
  while(!InitializeFlash())
  {
    #ifdef PRINT_DEBUG
    Serial.println("Flash error. Retry...");
    #endif
    delay(1000);
  }
  // get log file name
  FindNextSDFile();
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

  Serial.println("Running");
  logBtnDebounceStart = millis();
 }

void loop() 
{
  // if not logging, send timestamps to peers periodically
  if((isLogging!= LogState::ON) && (micros() - lastBroadcastTime > TIMESTAMP_BROADCAST))
  {
    BroadcastTimestamp();
    lastBroadcastTime = micros();
  }
  // check log button state
  CheckLogButton();
}
//
// check for log button press/release to change state
//
void CheckLogButton()
{
  unsigned long curTime = millis();
  // check for log button press with 1 sec debounce
  
  if((digitalRead(LOG_BUTTON) == LOW) && (logBtnDebounceStart == 0))
  {
    Serial.println("LOG_BUTTON LOW (start)");
    logBtnDebounceStart = millis();
  }
  if((digitalRead(LOG_BUTTON) == HIGH) && (logBtnDebounceStart != 0))
  {
    Serial.println("LOG_BUTTON HIGH");
    if(millis() - logBtnDebounceStart >= 1000)
    {
      #ifdef PRINT_DEBUG
      Serial.print("CheckLogButton on core ");
      Serial.print(xPortGetCoreID());
      Serial.print(" Button release, send logging change to ");
      Serial.println((isLogging == LogState::ON ? "LogState::OFF" : "LogState::ON"));
      #endif
      SendLoggingChange((isLogging == LogState::ON ? LogState::OFF : LogState::ON));
    }
    logBtnDebounceStart = 0;
  }
}
//
// initialize SD card
//
bool InitializeSD()
{
  if (!sd.begin(chipSelect, SPI_SPEED)) 
  {
    if (sd.card()->errorCode()) 
    {
      Serial.print(F("\nSD initialization failed"));
      Serial.print(F("errorCode: "));Serial.print((int)showbase, HEX);
      Serial.print(F(" "));
      Serial.print(int(sd.card()->errorCode()));
      Serial.print(F(" errorData: "));Serial.print(int(sd.card()->errorData()));
      Serial.println((int)noshowbase);
      return false;
    }
    if (sd.vol()->fatType() == 0) 
    {
      Serial.print(F("No valid FAT16/FAT32 partition - reformat card\n"));
      return false;
    }
    Serial.print(F("Unknown error\n"));
    return false;
  }

  uint32_t size = sd.card()->sectorCount();
  if (size == 0) 
  {
    Serial.print(F("Can't determine the SD card size.\n"));
    return false;
  }
  Serial.println(F("\nSD card initialized"));
  uint32_t sizeMB = 0.000512 * size + 0.5;
  Serial.print(F("\tVolume is FAT"));Serial.println(int(sd.vol()->fatType()));
  Serial.print(F("\tCard size: "));Serial.print(sizeMB);Serial.println(F("MB"));
  Serial.print(F("\tCluster size (bytes): "));Serial.println( sd.vol()->bytesPerCluster());

  if ((sizeMB > 1100 && sd.vol()->sectorsPerCluster() < 64) || 
      (sizeMB < 2200 && sd.vol()->fatType() == 32)) 
  {
    Serial.println(F("\tWARNING - Reformat for best performance"));
  }
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
  char msgType = isLogging == LogState::ON ? LOGGING_BEGIN : LOGGING_END;
  // logging started
  if(isLogging == LogState::ON)
  {
    // open or create file - truncate existing file.
    if(!OpenFlash(nextLogName))
    {
      #ifdef PRINT_DEBUG
      Serial.print("Failed to open Flash file ");
      Serial.print(nextLogName);
      Serial.println(", logging halted");
      #endif
      isLogging = LogState::OFF;
      return;
    }
    #ifdef PRINT_DEBUG
    Serial.print(micros());
    Serial.print(" START logging to ");
    Serial.println(nextLogName);
    #endif
  }
  // logging ended
  else
  {
    // close current file
    double s = CloseFlash(nextLogName);
    // find next file
    #ifdef PRINT_DEBUG
    Serial.print(micros());
    Serial.print(" END logging to file ");
    Serial.print(nextLogName);
    Serial.print(" (");
    Serial.print(s);
    Serial.print(") bytes");
    Serial.println();
    CopyFlashToSD(nextLogName);
    DeleteFlash(nextLogName);
    #endif
    FindNextSDFile();    
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
void SendLoggingState(const uint8_t *macAddr)
{
  AddPeer(macAddr);
  char msgType = isLogging == LogState::ON ? LOGGING_BEGIN : LOGGING_END;
  uint8_t result = esp_now_send(macAddr, (const uint8_t*)&msgType, sizeof(msgType));
  #ifdef PRINT_DEBUG
  Serial.print("Send current hub logging state "); 
  Serial.print(isLogging == LogState::ON ? "LOGGING to " : "IDLE to ");
  snprintf(msgOut, sizeof(msgOut), "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tresult\t%d\n",
           micros(), 
           macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5], result);
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
//
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if(isLogging == LogState::ON)
  {
    // timestamp request or heartbeat received while hub logging on - send start logging to leaf
    if((data[0] == TIMESTAMP_TYPE) ||
       (data[0] == HEARTBEAT_TYPE))
    {
      SendLoggingState(mac_addr);        
    }
    // store sensor data
    else
    {
      WriteFlash(data, data_len);
    }
  }
  // data received while logging is off
  else
  {
    // timestamp request or heartbeat 
    if(data[0] == TIMESTAMP_TYPE)
    {
      memcpy(&timeStampPacket, data, data_len);
      sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tTIMESTAMP REQt%010lu\n",
           micros(), 
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
           timeStampPacket.packet.timeStamp);
      Serial.print(msgOut);
      AddPeer(mac_addr);
      BroadcastTimestamp(mac_addr);
    }
    else if (data[0] == HEARTBEAT_TYPE)
    {
      memcpy(&timeStampPacket, data, data_len);
      sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tHEARTBEAT\t%010lu\n",
           micros(), 
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
           timeStampPacket.packet.timeStamp);
      Serial.print(msgOut);
      AddPeer(mac_addr);
    }
    // data received while hub logging off - send stop logging to leaf    
    else
    {
      sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\t%c\n",
           micros(), 
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
           (char)data[0]);
      Serial.print(msgOut);
      SendLoggingState(mac_addr);        
    }
  }
}
//
//
//
void AddPeer(const uint8_t* mac_addr)
{
  bool peerFound = false;
  bool peerValid = false;
  for(int macIdx = 0; macIdx < 6; macIdx++)
  {
    if((int)mac_addr[macIdx] != 0)
    {
      peerValid =  true;
      break;
    }
  }
  if(!peerValid)
  {
    return;
  }
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
  if(peerFound)
  {
    return;
  }
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
    sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tadded as peer %d\n", 
             micros(), 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
             (peerCnt));
  }
  else
  {
    sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tNOT added as peer ERROR %d\n", 
             rcvTime, 
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], 
             result);
  }
  Serial.print(msgOut);
  #endif
}
//
//
//
void BroadcastTimestamp()
{
  // setup timestamp
  uint8_t result;
  timeStampPacket.packet.msgType = TIMESTAMP_TYPE;
  timeStampPacket.packet.timeStamp = micros();
  // send to all known peers
  result = esp_now_send(0, &timeStampPacket.dataBytes[0], sizeof(timeStampPacket));
  #ifdef PRINT_DEBUG
  for(int peerIdx = 0; peerIdx < peerCnt; peerIdx++)
  {
    sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tBroadcast timestamp\t%lu\n",
             timeStampPacket.packet.timeStamp, 
             peers[peerIdx].peer_addr[0], peers[peerIdx].peer_addr[1], peers[peerIdx].peer_addr[2], peers[peerIdx].peer_addr[3], peers[peerIdx].peer_addr[4], peers[peerIdx].peer_addr[5],
             timeStampPacket.packet.timeStamp);
    Serial.print(msgOut);
  }
  #endif
}
//
//
//
void BroadcastTimestamp(const uint8_t* mac_addr)
{
  // setup timestamp
  uint8_t result;
  timeStampPacket.packet.msgType = TIMESTAMP_TYPE;
  timeStampPacket.packet.timeStamp = micros();
  // send to all known peers
  result = esp_now_send(mac_addr, &timeStampPacket.dataBytes[0], sizeof(timeStampPacket));
  #ifdef PRINT_DEBUG
  sprintf(msgOut, "%010lu\t%02X:%02X:%02X:%02X:%02X:%02X\tBroadcast timestamp\t%lu\n",
           timeStampPacket.packet.timeStamp, 
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
           timeStampPacket.packet.timeStamp);
  Serial.print(msgOut);
  #endif
}
//
// get next sequential log file name
//
//
// find the next log_nnnn.ylg file in series
//
void FindNextSDFile()
{
  int logIdx = 0;
  while(true)
  {
    sprintf(nextLogName, "/log_%04d.ylg", logIdx);
    if(!sd.exists(nextLogName))
    {
      break;
    }
    logIdx++;
  }
  Serial.print("Next file ");
  Serial.println(nextLogName);
}
//
// LittleFS (Flash) functions
// 
// 
// initialize - format on first use
//
bool InitializeFlash()
{
  if(!LITTLEFS.begin(FORMAT_LITTLEFS_IF_FAILED))
  {
      Serial.println("Flash mount Failed");
      return false;
  }
  Serial.println(F("\nFlash initialized"));
  Serial.print(F("\tAvailable "));
  Serial.println(LITTLEFS.totalBytes()/1000.0, 3);
  Serial.print(F("\tUsed "));
  Serial.print(LITTLEFS.usedBytes()/1000.0, 3);
  Serial.println(F("KB"));
  int levels = 4;
  File root = LITTLEFS.open("/");
  if(!root)
  {
    Serial.println("- failed to open directory");
    return true;
  }
  if(!root.isDirectory())
  {
    Serial.println(" - not a directory");
    return true;
  }
  File file = root.openNextFile();
  while(file)
  {
    if(file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels)
      {
        ListFlashDir(file.name(), levels -1);
      }
    }
    else 
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
  return true;
}
void ListFlashDir(const char * dirname, uint8_t levels)
{
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = LITTLEFS.open(dirname);
  if(!root)
  {
    Serial.println("- failed to open directory");
    return;
  }
  if(!root.isDirectory())
  {
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file)
  {
    if(file.isDirectory())
    {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels)
      {
        ListFlashDir(file.name(), levels -1);
      }
    }
    else 
    {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("\tSIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}
//
// write to open flash file
//
int WriteFlash(const uint8_t * msg, size_t msgSize)
{
  return flashFile.write(msg, msgSize);
}
//
// delete flash file
//
void DeleteFlash(const char * path)
{
  Serial.printf("Delete FLASH file: %s ", path);
  if(!LITTLEFS.remove(path))
  {
      Serial.print("- delete failed");
  }
  Serial.println();
}
//
// copy flash file to SD
//
void CopyFlashToSD(const char * path)
{
  Serial.printf("Copy file: %s from FLASH to SD ", path);

  flashFile = LITTLEFS.open(path);
  if(!flashFile || flashFile.isDirectory()){
      Serial.println("- failed to open FLASH file for read");
      return;
  }
  if (!logFileSD.open(path, O_RDWR | O_CREAT | O_TRUNC)) 
  {
    sd.errorHalt(&Serial, F(" - failed to open SD file for write"));
    return;
  }

  static uint8_t buf[512];
  size_t len = flashFile.size();
  while(len)
  {
    size_t toRead = len;
    if(toRead > 512)
    {
      toRead = 512;
    }
    flashFile.read(buf, toRead);
    logFileSD.write(buf, toRead);
    Serial.print(".\r");
    len -= toRead;
  }
  flashFile.close();  
  logFileSD.close();
  Serial.println();
  Serial.printf("Done! file: %s copied from FLASH to SD\n", path);
}
//
//
//
bool OpenFlash(const char * path)
{
  flashFile = LITTLEFS.open(path, FILE_WRITE);
  if(!flashFile)
  {
      Serial.println("- failed to open file for writing");
      return false;
  }
  return true;
}
unsigned long CloseFlash(const char * path)
{
  flashFile.close();
  flashFile = LITTLEFS.open(path);
  unsigned long len = flashFile.size();
  flashFile.close();
  return len;
}
