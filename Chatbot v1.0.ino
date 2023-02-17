/*-----------------------------/START OF CODE/--------------------------------------------------------------------//
  ChatBot
  This code runs a ESP8266 that handles switching on and off a computer. It is set to stay
  awake for only a minute unless it receives commands or is commanded to stay awake. 
  It can turn on and off a computer, as well as perform basic checks on the computer status
  and update the user about the boar operation and connection detals. It writes data on an 
  as needed basis to the RTC memory bank depending on any situations that have changed. 

  It connects to an MQTT server and only handles QoS 0 data packages when connected. It will
  notify the MQTT client when it connects, and when it is soon to be heading to sleep. It 
  will always send feedback commands when it receives commands, including unknown commands. 

  This is an ongoing project and will be updated as need be. 
  
  V1.0 - initial run, works and tested to full functionality


  Last Update: 16 Feb 2023 3:58PM PST -- Ryan Sass
  */

/////////////////////////Declarations////////////////////////////////////////////////////////////////////
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <time.h>
#include <TZ.h>

#define MSG_BUFFER_SIZE (110)
#define PORT (1883)
#define HEARTBEAT (5 * (60000))  // value in minutes
#define WakeMode RFMode
#define MAC_SIZE 6
#define MAX_TZ 54                        //maximum number of TZ's listed
#define STATUS_LOOPS 10                  // used for size and loop length for state[payloadIndex], which records the state of the output
#define MAX_MILLIS 4200000000
#ifndef WAKE_RF_DEFAULT
#define WAKE_RF_DEFAULT RF_DEFAULT
#define WAKE_RFCAL RF_CAL
#define WAKE_NO_RFCAL RF_NO_CAL
#define WAKE_RF_DISABLED RF_DISABLED
#endif
#ifndef STASSID
#define STASSID "remotewhyfi" //MUST UPDATE THIS PRIOR TO RUNNING, USE LOCAL WIFI SSID
#define STAPSK "sillysillyfruit" //MUST UPDATE THIS PRIOR TO RUNNING, USE LOCAL WIFI PASSWORD
#endif
#define COMMAND_1 "Hello"             //Status Check light
#define COMMAND_2 "Time"                 // turn on computer
#define COMMAND_3 "Location"                // turn off computer
#define COMMAND_4 "Origin"              // press reset button
#define COMMAND_5 "Sing"          //hold off button 10s
#define COMMAND_6 "Time Zone:"           // press power button
#define COMMAND_7 "Time of Day"           // press power button
#define COMMAND_8 "Heart Beat"           // press power button
#define COMMAND_9 "Wifi Status"           // press power button
#define COMMAND_10 ""           // press power button
#define COMMAND_11 "Board Restart"           // press power button
#define COMMAND_12 "Howdy"
#define COMMAND_13 "question"
#define COMMAND_14 "hi"


//REMEMBER THERE ARE 3 PLACES TO ADD NEW COMMANDS: messageConversion(), processCommand(), and commandCode()
const char *subscribeTopic = "chatbot/demo/webpage/message";           //command input channel
const char *publishTopic = "chatbot/demo/webpage/response";           //command output channel
const char *publishInteractCount = "remote/controller/sw1999";       //interaction count
const char *ssid = STASSID;
const char *password = STAPSK;
const char *mqttServerAddress = "91.121.93.94";  // test.mosquitto.org
//const char* mqttServerAddress = "2001:41d0:1:925e::1"; //test.mosquitto.org ipv6 address
//const char* mqttServerAddress = "broker.mqtt-dashboard.com"; //original

typedef struct {
  uint32_t crc32;            //4 bytes
  uint8_t channel;           //1 byte, 5 in total
  uint8_t ap_mac[MAC_SIZE];  //MAC_SIZE (6) bytes, 11 in total
  uint8_t timeZone;          //1 byte, 12 in total
  uint8_t padding;           //1 byte, 13 in total
} rtcStore;
rtcStore rtcData;

IPAddress ip(192, 168, 0, 253);  //pick an IP outside the DHCP range
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(75, 75, 75, 75);

char globalMessage[MSG_BUFFER_SIZE],
     globalMessageAlternate[MSG_BUFFER_SIZE],
     globalMessageUTC[MSG_BUFFER_SIZE],
     globalMessageAwake[MSG_BUFFER_SIZE];

unsigned long lastMessage = 0,
              awakeStartTime = 0,
              originalStartTime = 0,
              timeAwake = 60000;  //1 min awake default

unsigned int travelTimeStopTime = 0;  //travel time stop point
int state[STATUS_LOOPS];
int SLEEPTIME = 9,    //9 min sleep default
  sleepTime = 9,      //default sleep time in minutes, will be converted to micro seconds (us)
  timeZone = 0,       //defaults to 0, which is set to PST/PDT
  heartBeatCount = 0,        //publishing code
  sleepCountdownCode = 1,      //sleep countdown code
  mqttReconnectAttempts = 0,  //mqtt reconnect attempts
  globalPayload = 0,           //message information payload
  payloadIndex = 0,              //index
  interactCount = 0;
bool printSpace = false,
     getInteractCount = false,
     rtcValid = false,
     dayTime = true,
     wifiMemUpdate = false,
     timeZoneMemUpdate = false,
     noTime = false,
     noPublish = false,
     firstLoop = true,
     deepNight = false,
     ledState = false,
     clearChannel = false;

//DEBUG FLAGS //
///normal settings for normal operation///
/*/bool outputSerial = false;
    #define GETTIME true
    #define SLEEPALLOW true//*/
///full debug settings////
/*/bool outputSerial = true;
    #define GETTIME false
    #define SLEEPALLOW false//*/
///custom debug settings////
/**/ bool outputSerial = true;
#define GETTIME true
#define SLEEPALLOW true  //*/

WiFiClient espClient;
PubSubClient client(espClient);

char *currentTime(bool local = false, int minutes = 0);
void publish(char *display, int heartbeatCount = -1);
void print(String toPrint = "");
void println(String toPrint = "");
void reportCommandClear(int ignore = 0);

/////////////////////////RTC FUNCTIONS///////////////////////////////////////////////////////////////////
void checkRTC() {
  rtcValid = false;
  if (ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData))) {
    //calculate the  CRC of what was just read from RTC memory, but skip the first 4 bytes as that's the checksum
    delay(1);
    uint32_t crc = calculateCRC32(((uint8_t *)&rtcData) + 4, sizeof(rtcData) - 4);
    if (crc == rtcData.crc32) {
      rtcValid = true;
    }
  }
}

void rtcMemUpdate() {
  rtcData.timeZone = timeZone;
  rtcData.channel = WiFi.channel();  
  memcpy(rtcData.ap_mac, WiFi.BSSID(), 6);  //copy 6 bytes of BSSID (AP's MAC address)  
  uint32_t crc = calculateCRC32(((uint8_t *)&rtcData) + 4, sizeof(rtcData) - 4);
  rtcData.crc32 = crc;
  ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData));
  delay(100);
  return;
}

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t count = 0x80; count > 0; count >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & count) bit = !bit;
      crc <<= 1;
      if (bit) crc ^= 0x04c11db7;
    }
  }
  return crc;
}

/////////////////////////CONFIG FUNCTIONS ///////////////////////////////////////////////////////////////
void configure() {  
  Serial.begin(115200);
  Serial.setTimeout(2000);  
  delay(50);
  println();
  if (rtcValid) println("rtcValid!");
  else println("rtcInValid!");  //
  if (rtcValid) {
    timeZone = rtcData.timeZone;  //reestablishing the local time zone
  }  
  else
    timeZone = 66;  
  pinMode(BUILTIN_LED, OUTPUT);  // Initialize the BUILTIN_LED pin as an output
  led_OFF();
}  // default turn LED off

int boolToInt(bool input) {
  if (input) return 1;
  else return 0;
}

bool intToBool(int input) {
  if (input == 0) return false;
  else return true;
}

String boolToString(bool input) {
  if (input) return "TRUE";
  else return "FALSE";
}

int asciiToInt(int input) {
  switch (input) {
    case 48:
      return 0;
    case 49:
      return 1;
    case 50:
      return 2;
    case 51:
      return 3;
    case 52:
      return 4;
    case 53:
      return 5;
    case 54:
      return 6;
    case 55:
      return 7;
    case 56:
      return 8;
    case 57:
      return 9;
    default:
      return -1;
  }
}

void restart() {
  ESP.restart();
  client.publish(subscribeTopic, "", true);
  publish("ESP.restart() Called...?????");
}

/////////////////////////CONNECT FUNCTIONS///////////////////////////////////////////////////////////////
void startWiFi() {
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(10);
  WiFi.forceSleepWake();
  delay(10);
  //Disable the WiFi persistence. The ESP8266 will not load and save WiFi settings unnecessarily in the flash memory.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, dns, gateway, subnet);
  println();
  print("Connecting to ");
  print(ssid);
  if (rtcValid) WiFi.begin(ssid, password, rtcData.channel, rtcData.ap_mac, true);
  else WiFi.begin(ssid, password);
  unsigned long loopStart = millis();
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    retries++;
    delay(100);
    print(".");
    if (retries >= 100 && retries <= 104) {
      //after 20s quick connect is not functioning reset WiFi and try regular connection
      println("Resetting WiFi...");
      WiFi.disconnect();
      delay(100);
      WiFi.forceSleepBegin();
      delay(100);
      WiFi.forceSleepWake();
      delay(50);
      WiFi.begin(ssid, password);
      retries = 105;
    }
    if (retries >= 600) {
      //giving up after 60 seconds and going back to sleep
      println("");
      println("Connection Timeout. Restarting.");
      WiFi.disconnect(true);
      delay(100);
      WiFi.mode(WIFI_OFF);
      delay(50);
      noPublish = true;
      restart();
      break;
    }
    
  }
  if (retries > 0) println();
  print("Conntected to WiFi: ");
  println(ssid);
  println("IP Address: ");
  if (outputSerial) Serial.println(WiFi.localIP());
  if (!rtcValid) wifiMemUpdate = true;
}  

void setupMQTT() {
  client.setServer(mqttServerAddress, PORT);
  client.setCallback(callback);
}

void reconnect() {
  unsigned long loopStart = millis();
  while (!client.connected()) {  // Loop until reconnection
    mqttReconnectAttempts++;
    print("Attempting MQTT connection on ");
    print(mqttServerAddress);
    print("...");
    String clientId = "ESP8266Client-";  // Create a random client ID
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), publishTopic, 0, true, "From MQTT server: Device disconnected unexpectedly!")) {  // Attempt to connect
      println("connected!");      
      snprintf(globalMessage, MSG_BUFFER_SIZE, "Connection Successful! at (TZ#: %d) %s %s", timeZone, UTCoffset(timeZone), currentTime(true));
      client.publish(publishTopic, globalMessage, true);  // Once connected, publish an announcement...
      client.subscribe(subscribeTopic);           // ... and resubscribe
      client.subscribe(publishInteractCount);
      if (firstLoop) {                        // perform things during the first go of the main loop
        //client.publish(publishTopic,"", true); //moved this clearing to above, subscribe and unsubscribe, and handle any errant messages in callback function        
        originalStartTime = millis();
        if (!rtcValid) {
          publish("RTC Invalid");
          rtcMemUpdate();
        }        
        firstLoop = false;
      }
    } else {
      print("failed, rc=");
      print(String(client.state()));
      println(" try again in 5 seconds");
      mqttConnFailBlink();
    } 
  }
  printSp();
  awakeStartTime += (millis() - loopStart);
}

unsigned long heartbeat() {
  if (firstLoop) {
    println("Heartbeat 0");
    println();
  } else publish("", heartBeatCount++);
  println();
  heartbeatBlink();
  return millis();
}

/////////////////////////TIME FUNCTIONS//////////////////////////////////////////////////////////////////
void setLocalTimeZone(int zone) {
  switch (zone) {  //remember to update UTCoffset() function and MAX_TZ definition when any changes are made here
    case 0:
      //the global servers: "pool.ntp.org", "time.nist.gov"
      configTime(TZ_America_Los_Angeles, "pool.ntp.org", "time.nist.gov");
      break;
    case 1:
      configTime(TZ_America_Phoenix, "pool.ntp.org", "time.nist.gov");
      break;
    case 2:
      configTime(TZ_America_Denver, "pool.ntp.org", "time.nist.gov");
      break;
    case 3:
      configTime(TZ_America_Chicago, "pool.ntp.org", "time.nist.gov");
      break;
    case 4:
      configTime(TZ_America_New_York, "pool.ntp.org", "time.nist.gov");
      break;
    case 5:
      configTime(TZ_America_Anchorage, "pool.ntp.org", "time.nist.gov");
      break;
    case 6:
      configTime(TZ_Pacific_Honolulu, "pool.ntp.org", "time.nist.gov");
      break;
    case 7:
      configTime(TZ_Etc_GMTp12, "pool.ntp.org", "time.nist.gov");
      break;
    case 8:
      configTime(TZ_Pacific_Midway, "pool.ntp.org", "time.nist.gov");
      break;
    case 9:
      configTime(TZ_Pacific_Gambier, "pool.ntp.org", "time.nist.gov");
      break;
    case 10:
      configTime(TZ_Pacific_Pitcairn, "pool.ntp.org", "time.nist.gov");
      break;
    case 11:
      configTime(TZ_America_Edmonton, "pool.ntp.org", "time.nist.gov");
      break;
    case 12:
      configTime(TZ_America_Mexico_City, "pool.ntp.org", "time.nist.gov");
      break;
    case 13:
      configTime(TZ_America_Cayman, "pool.ntp.org", "time.nist.gov");
      break;
    case 14:
      configTime(TZ_America_Santiago, "pool.ntp.org", "time.nist.gov");
      break;
    case 15:
      configTime(TZ_America_Puerto_Rico, "pool.ntp.org", "time.nist.gov");
      break;
    case 16:
      configTime(TZ_America_Sao_Paulo, "pool.ntp.org", "time.nist.gov");
      break;
    case 17:
      configTime(TZ_Atlantic_Cape_Verde, "pool.ntp.org", "time.nist.gov");
      break;
    case 18:
      configTime(TZ_Europe_London, "pool.ntp.org", "time.nist.gov");
      break;
    case 19:
      configTime(TZ_Atlantic_Reykjavik, "pool.ntp.org", "time.nist.gov");
      break;
    case 20:
      configTime(TZ_Europe_Paris, "pool.ntp.org", "time.nist.gov");
      break;
    case 21:
      configTime(TZ_Africa_Casablanca, "pool.ntp.org", "time.nist.gov");
      break;
    case 22:
      configTime(TZ_Europe_Kiev, "pool.ntp.org", "time.nist.gov");
      break;
    case 23:
      configTime(TZ_Africa_Cairo, "pool.ntp.org", "time.nist.gov");
      break;
    case 24:
      configTime(TZ_Europe_Moscow, "pool.ntp.org", "time.nist.gov");
      break;
    case 25:
      configTime(TZ_Asia_Dubai, "pool.ntp.org", "time.nist.gov");
      break;
    case 26:
      configTime(TZ_Asia_Karachi, "pool.ntp.org", "time.nist.gov");
      break;
    case 27:
      configTime(TZ_Asia_Kolkata, "pool.ntp.org", "time.nist.gov");
      break;
    case 28:
      configTime(TZ_Asia_Bangkok, "pool.ntp.org", "time.nist.gov");
      break;
    case 29:
      configTime(TZ_Asia_Shanghai, "pool.ntp.org", "time.nist.gov");
      break;
    case 30:
      configTime(TZ_Asia_Tokyo, "pool.ntp.org", "time.nist.gov");
      break;
    case 31:
      configTime(TZ_Australia_Sydney, "pool.ntp.org", "time.nist.gov");
      break;
    case 32:
      configTime(TZ_Pacific_Guam, "pool.ntp.org", "time.nist.gov");
      break;
    case 33:
      configTime(TZ_Pacific_Norfolk, "pool.ntp.org", "time.nist.gov");
      break;
    case 34:
      configTime(TZ_Pacific_Kosrae, "pool.ntp.org", "time.nist.gov");
      break;
    case 35:
      configTime(TZ_Pacific_Auckland, "pool.ntp.org", "time.nist.gov");
      break;
    case 36:
      configTime(TZ_Pacific_Wake, "pool.ntp.org", "time.nist.gov");
      break;
    case 37:
      configTime(TZ_Australia_Broken_Hill, "pool.ntp.org", "time.nist.gov");
      break;
    case 38:
      configTime(TZ_Australia_Darwin, "pool.ntp.org", "time.nist.gov");
      break;
    case 39:
      configTime(TZ_Pacific_Fakaofo, "pool.ntp.org", "time.nist.gov");
      break;
    case 40:
      configTime(TZ_Pacific_Kiritimati, "pool.ntp.org", "time.nist.gov");
      break;
    case 41:
      configTime(TZ_Asia_Kathmandu, "pool.ntp.org", "time.nist.gov");
      break;
    case 42:
      configTime(TZ_America_St_Johns, "pool.ntp.org", "time.nist.gov");
      break;
    case 43:
      configTime(TZ_Atlantic_South_Georgia, "pool.ntp.org", "time.nist.gov");
      break;
    case 44:
      configTime(TZ_Asia_Dhaka, "pool.ntp.org", "time.nist.gov");
      break;
    case 45:
      configTime(TZ_Pacific_Marquesas, "pool.ntp.org", "time.nist.gov");
      break;
    case 46:
      configTime(TZ_Asia_Tehran, "pool.ntp.org", "time.nist.gov");
      break;
    case 47:
      configTime(TZ_Asia_Kabul, "pool.ntp.org", "time.nist.gov");
      break;
    case 48:
      configTime(TZ_Indian_Cocos, "pool.ntp.org", "time.nist.gov");
      break;
    case 49:
      configTime(TZ_Australia_Eucla, "pool.ntp.org", "time.nist.gov");
      break;
    case 50:
      configTime(TZ_Australia_Lord_Howe, "pool.ntp.org", "time.nist.gov");
      break;
    case 51:
      configTime(TZ_Pacific_Chatham, "pool.ntp.org", "time.nist.gov");
      break;
    case 52:
      configTime(TZ_America_Adak, "pool.ntp.org", "time.nist.gov");
      break;
    case 53:
      configTime(TZ_America_Nuuk, "pool.ntp.org", "time.nist.gov");
      break;
    case 54:
      configTime(TZ_Atlantic_Azores, "pool.ntp.org", "time.nist.gov");
      break;
    default:
      configTime(TZ_Etc_Universal, "pool.ntp.org", "time.nist.gov");
      timeZone = 66;
      break;
  }
}

String UTCoffset(int zone) {
  switch (zone) {  //remember to update setLocalTimeZone() function and MAX_TZ definition when any changes are made here
    case 0:        //America Los Angeles
      return "<UTC-8*>";
    case 1:  //America Phoenix
      return "<UTC-7>";
    case 2:  //America Denver
      return "<UTC-7*>";
    case 3:  //America Chicago
      return "<UTC-6*>";
    case 4:  //America New York
      return "<UTC-5*>";
    case 5:  //America Anchorage
      return "<UTC-9*>";
    case 6:  //America Honolulu
      return "<UTC-10>";
    case 7:  //America Internaltion Date Line West (TZ ETC GMT p12)
      return "<UTC-12>";
    case 8:  //Pacific Midway
      return "<UTC-11>";
    case 9:  //Pacific Gambier (Alaska)
      return "<UTC-9>";
    case 10:  //Pacific Pitcarin
      return "<UTC-8>";
    case 11:  //America Edmonton
      return "<UTC-7>";
    case 12:  //America Mexico City
      return "<UTC-6>";
    case 13:  //America Cayman
      return "<UTC-5>";
    case 14:  //America Santiago
      return "<UTC-4*>";
    case 15:  //America Puerto Rico
      return "<UTC-4>";
    case 16:  //America Sao Paulo
      return "<UTC-3>";
    case 17:  //Atlantic Cape Verde
      return "<UTC-1>";
    case 18:  //Europe London
      return "<UTC*>";
    case 19:  //Atlantic Reykjavik
      return "<UTC>";
    case 20:  //Europe Paris
      return "<UTC+1*>";
    case 21:  //Africa Casablanca
      return "<UTC+1>";
    case 22:  //Europe Kiev
      return "<UTC+2*>";
    case 23:  //Africa Cairo
      return "<UTC+2>";
    case 24:  //Europe Moscow
      return "<UTC+3>";
    case 25:  //Asia Dubai
      return "<UTC+4>";
    case 26:  //Asia Karachi
      return "<UTC+5>";
    case 27:  //Asia Kolkata
      return "<UTC+5:30>";
    case 28:  //Asia Bangkok
      return "<UTC+7>";
    case 29:  //Asia Shanghai
      return "<UTC+8>";
    case 30:  //Asia Tokyo
      return "<UTC+9>";
    case 31:  //Australia Sydney
      return "<UTC+10*>";
    case 32:  //Pacific Guam
      return "<UTC+10>";
    case 33:  //Pacific Norfolk Island
      return "<UTC+11*>";
    case 34:  //Pacific Kosrae
      return "<UTC+11>";
    case 35:  //Pacific Auckland
      return "<UTC+12*>";
    case 36:  //Pacific Wake Islands
      return "<UTC+12>";
    case 37:  //Australia Broken Hill
      return "<UTC+9:30*>";
    case 38:  //Australia Darwin
      return "<UTC+9:30>";
    case 39:  //Pacific Fakaofo
      return "<UTC+13>";
    case 40:  //Pacific Kiritimati
      return "<UTC+14>";
    case 41:  //Asia Kathmandu (Nepal)
      return "<UTC+5:45>";
    case 42:  //America St Johns
      return "<UTC-3:30*>";
    case 43:  //Atlantic South Georgia
      return "<UTC-2>";
    case 44:  //Asia Dhaka
      return "<UTC+6>";
    case 45:  //Pacific Marquesas
      return "<UTC-9:30>";
    case 46:  //Asia Tehran
      return "<UTC+3:30>";
    case 47:  //Asia Kabul
      return "<UTC+4:30>";
    case 48:  //Indian Cocos
      return "<UTC+6:30>";
    case 49:  //Australia Eucla
      return "<UTC+8:45>";
    case 50:  //Australia Lord Howe
      return "<UTC+10:30*>";
    case 51:  //Pacific Chatham
      return "<UTC+12:45*>";
    case 52:  //America Adak
      return "<UTC-10*>";
    case 53:  //America Nuuk
      return "<UTC-3*>";
    case 54:  //Atlantic Azores
      return "<UTC-1*>";
    default:  //TZ Etc Universal
      return "<UTC>";
  }
}

void setDateTime() {
  if (GETTIME) {
    setLocalTimeZone(timeZone);
    print("Waiting for NTP time sync: ");
    unsigned long loopStart = millis();
    //configTime(0,0,"pool.ntp.org");
    time_t now = time(nullptr);
    while (now < 1000) {
      //led_ON(); delay(50);
      if (outputSerial) print("*");
      now = time(nullptr);
      delay(1000);
      //led_OFF(); delay(50);
      if ((millis() - loopStart) > 60000) {
        noTime = true;
        break;
      }
    }
    struct tm timeinfo;
    if (outputSerial) {
      Serial.println();
      if (noTime) Serial.print("Server Timeout - No Time Synced!\n");
      else {
        Serial.printf("Current Universal Time: UTC %s", currentTime());
        localtime_r(&now, &timeinfo);
        Serial.printf("Current Local Time: %s %s", tzname[0], currentTime(true));
        Serial.println();
      }
    }
  } else noTime = true;
}

void determineDayNight() {
  if (SLEEPALLOW && GETTIME) {
    struct tm timeinfo;
    time_t now = time(nullptr);
    localtime_r(&now, &timeinfo);
    if ((timeinfo.tm_hour >= 23) || (timeinfo.tm_hour < 7)) {
      dayTime = false;
      if ((timeinfo.tm_hour >= 2) && (timeinfo.tm_hour <= 5)) deepNight=true;  
      else deepNight=false;                                                     
      return;
    }  
  }
}

char *currentTime(bool local, int minutes) {
  if (noTime) return "Unknown Time!\n";
  unsigned long loopStart = millis();
  time_t now = time(nullptr), newTime;
  while (now < 1000) {
    delay(50);
    print("-");
    now = time(nullptr);
    if ((millis() - loopStart) > (60000 / 10)) {
      break;
    }
  }
  struct tm timeinfo;
  size_t seconds = minutes * 60;
  newTime = now + seconds;
  memset(&timeinfo, '\0', sizeof(struct tm));
  if (local) localtime_r(&newTime, &timeinfo);
  else gmtime_r(&newTime, &timeinfo);
  return asctime(&timeinfo);
}

/////////////////////////MQTT FUNCTIONS//////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int length) {
  String payloadString, payloadClean, localMessage;
  for (int count = 0; count < length; count++) payloadString += (String(payload[count]));  
  if (strcmp(topic, publishInteractCount) == 0) {  //   
    getInteractCount = true;
    interactCount = messageConversionCount(payloadString, length);
    client.unsubscribe(publishInteractCount);
    return;
  }  
  payloadClean = sanatizePayload(payloadString, length);
  length = payloadLength(payloadClean, length);
  payloadString = messageConversion(payloadClean, length);
  String truncatedMessage = payloadString.substring(0, length);
  payloadClean = (char *)topic;
  if (payloadString.equalsIgnoreCase(F(COMMAND_6))) {
    timeZone = globalPayload;
    localMessage = "Message from [" + payloadClean + "]: " + truncatedMessage + " " + String(globalPayload) + " at " + tzname[0] + " " + currentTime(true);
  } else localMessage = "Message from [" + payloadClean + "]: " + truncatedMessage + " at " + tzname[0] + " " + currentTime(true);
  print((char *)localMessage.c_str());
  acknowledgeCmdBlink();
  processCommand(payloadString);
}

int characterIndex(int value) {
  if (32 <= value && value <= 99) return 2;
  else return 3;
}

int payloadLength(String payload, unsigned int length) {
  int payloadIndex = 0, nextIndex, refind, result, spaceCount = 0;
  for (int index = 0; index < length; index++) {
    result = (String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt();
    if (result == 32) {
      if (((String(payload[payloadIndex + 2]) + String(payload[payloadIndex + 3])).toInt() == 32) || (length - index) <= 1) {
        spaceCount++;
        refind = index + 1;
        nextIndex = payloadIndex + 2;
        //count extra spaces in front of initial space
        while (refind < length) {
          result = (String(payload[nextIndex]) + String(payload[nextIndex + 1])).toInt();
          if (result != 32) break;
          else spaceCount++;
          nextIndex += 2;
          refind++;
        }
        return (length - spaceCount);
      }
    }  //if the tail spaces
    payloadIndex += characterIndex((String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt());
    spaceCount = 0;
  }
  return (length - spaceCount);
}

String sanatizePayload(String payload, unsigned int length) {
  int payloadIndex = 0, nextIndex, nextPayloadIndex, refind, result, extraDebugSpace = 0, spaceCount = 0, numberFromPayload = 0;
  bool save = true;
  char charOne, charTwo;
  for (int index = 0; index < length; index++) {
    result = (String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt();
    if (result == 32) {
      if (save) {
        charOne = payload[payloadIndex];
        charTwo = payload[payloadIndex + 1];
        save = false;
      }
      spaceCount++;
      refind = index + 1;
      nextIndex = payloadIndex + 2;
      //count extra spaces in front of initial space
      while (refind < length) {
        result = (String(payload[nextIndex]) + String(payload[nextIndex + 1])).toInt();
        if (result != 32) break;
        else spaceCount++;
        nextIndex += 2;
        refind++;
      }
      nextPayloadIndex = payloadIndex;
      if (index == 0) extraDebugSpace = 0;
      else {
        extraDebugSpace = 1;
        nextPayloadIndex += 2;
      }
      spaceCount -= extraDebugSpace;
      if ((refind < length) && (spaceCount > 0)) {  //if not the tail spaces
        for (refind; refind < length; refind++) {
          payload[nextPayloadIndex] = payload[nextIndex];
          payload[nextPayloadIndex + 1] = payload[nextIndex + 1];
          numberFromPayload = characterIndex((String(payload[nextIndex]) + String(payload[nextIndex + 1])).toInt());
          if (numberFromPayload == 3) payload[nextPayloadIndex + 2] = payload[nextIndex + 2];
          nextPayloadIndex += numberFromPayload;
          nextIndex += numberFromPayload;
        }
        for (refind = length - spaceCount; refind < length; refind++) {
          payload[nextPayloadIndex] = charOne;
          payload[nextPayloadIndex + 1] = charTwo;
          nextPayloadIndex += 2;
        }
      } else if (refind >= length) return payload;
    }
    payloadIndex += characterIndex((String(payload[payloadIndex]) + String(payload[payloadIndex + 1])).toInt());
    spaceCount = 0;
  }
  return payload;
}

unsigned long messageConversionCount(String message, unsigned int length){
  String payloadString, localTimeZone;
  int messageIndex = 0, nextIndex = 0, indexRef = 0, result, payloadTimeZone = -1;
  for (int index = 0; index < length; index++) {
    nextIndex = characterIndex((String(message[messageIndex]) + String(message[messageIndex + 1])).toInt());
    if (nextIndex == 2) result = (String(message[messageIndex]) + String(message[messageIndex + 1])).toInt();
    else result = (String(message[messageIndex]) + String(message[messageIndex + 1]) + String(message[messageIndex + 2])).toInt();
    messageIndex += nextIndex;
    payloadString += char(result);
  }
  return payloadString.toInt();
}

String messageConversion(String message, unsigned int length) {
  String payloadString, localTimeZone;
  int messageIndex = 0, nextIndex = 0, indexRef = 0, result, payloadTimeZone = -1;
  for (int index = 0; index < length; index++) {
    if (payloadString.equalsIgnoreCase(F(COMMAND_6))) {
      int digits = 2;      
      result = (String(message[messageIndex]) + String(message[messageIndex + 1])).toInt();
      nextIndex = messageIndex;
      indexRef = index;
      if (result == 32) {
        nextIndex += 2;
        indexRef++;
      }
      for (int q = 0; q <= digits; q++) {  //process digits
        if (indexRef < length) {
          result = (String(message[nextIndex]) + String(message[nextIndex + 1])).toInt();
          if (48 <= result && result <= 57) {
            localTimeZone += char(result);
            nextIndex += 2;
            indexRef++;
          } else break;
        } else {
          if (q != 0) payloadTimeZone = localTimeZone.toInt();
          break;
        }
      }
      //check for more digits
      if (indexRef < length) {
        if ((String(message[nextIndex]) + String(message[nextIndex + 1])).toInt() == 32) {
          indexRef++;
          nextIndex += 2;
          if (indexRef < length)
            if ((String(message[nextIndex]) + String(message[nextIndex + 1])).toInt() == 32) payloadTimeZone = localTimeZone.toInt();
            else payloadTimeZone = localTimeZone.toInt();
        }
      }
      globalPayload = payloadTimeZone;
      if (payloadString.equalsIgnoreCase(F(COMMAND_6))) {
        if ((0 <= payloadTimeZone) && (payloadTimeZone <= MAX_TZ)) return COMMAND_6;
      }
    }    
    nextIndex = characterIndex((String(message[messageIndex]) + String(message[messageIndex + 1])).toInt());
    if (nextIndex == 2) result = (String(message[messageIndex]) + String(message[messageIndex + 1])).toInt();
    else result = (String(message[messageIndex]) + String(message[messageIndex + 1]) + String(message[messageIndex + 2])).toInt();
    messageIndex += nextIndex;
    payloadString += char(result);
    if(result == 63) return COMMAND_13;
  }
  if (payloadString.equalsIgnoreCase(F(COMMAND_1))) return COMMAND_1;  //COMMAND LIST HERE
  if (payloadString.equalsIgnoreCase(F(COMMAND_2))) return COMMAND_2;
  if (payloadString.equalsIgnoreCase(F(COMMAND_3))) return COMMAND_3;
  if (payloadString.equalsIgnoreCase(F(COMMAND_4))) return COMMAND_4;
  if (payloadString.equalsIgnoreCase(F(COMMAND_5))) return COMMAND_5;
  if (payloadString.equalsIgnoreCase(F(COMMAND_6))) return COMMAND_6;
  if (payloadString.equalsIgnoreCase(F(COMMAND_7))) return COMMAND_7;
  if (payloadString.equalsIgnoreCase(F(COMMAND_8))) return COMMAND_8;
  if (payloadString.equalsIgnoreCase(F(COMMAND_9))) return COMMAND_9;
  if (payloadString.equalsIgnoreCase(F(COMMAND_10))) return COMMAND_10;
  if (payloadString.equalsIgnoreCase(F(COMMAND_11))) return COMMAND_11;
  if (payloadString.equalsIgnoreCase(F(COMMAND_12))) return COMMAND_12;
  if (payloadString.equalsIgnoreCase(F(COMMAND_13))) return COMMAND_13;
  if (payloadString.equalsIgnoreCase(F(COMMAND_14))) return COMMAND_14;
  return payloadString;
}

void processCommand(String command) {
  printSpace = true;
  String commandMessage;
  String truncatedMessage = command.substring(0, 18);
  if (truncatedMessage == "") truncatedMessage = "[Blank]";  
  int commandCodeResult = commandCode(command);  
  if(commandCodeResult == 14) commandCodeResult = 1;
  if(clearChannel && commandCodeResult == 10) return;
  clearChannel = false;
  switch (commandCodeResult) {        
    case 1: // hello
      switch(random(3)){
        case 0:
          commandMessage = "'Howdy.'";  
          break;
        case 1:
          commandMessage = "'Greetings!'";  
          break;
        default:
          commandMessage = "'Well hello there!'";  
          break;
      }
      publish((char *)commandMessage.c_str());      
        break;
    case 2:  //Time Check
      commandMessage = "Current Time Zone #" + String(timeZone) + " (" + tzname[0] + ") " + UTCoffset(timeZone);      
      publish((char *)commandMessage.c_str());
      break;
    case 3: // location
        switch(random(3)){
        case 0:
          commandMessage = "'I'm located in the SF Bay Area!'";  
          break;
        case 1:
          commandMessage = "'I'm from Sunny California... Yay Area!'";  
          break;
        default:
          commandMessage = "'I'm sitting on Ryan's Desk hoping you hire him!'";  
          break;      }        
        publish((char *)commandMessage.c_str());
        break;
    case 4: //origin
        switch(random(3)){
        case 0:
          commandMessage = "'I was programmed by Ryan Sass. He's a great engineer!'";  
          break;
        case 1:
          commandMessage = "'I was built somewhere in Asia, came to Ryan Sass via Amazon and he gave me instructions to perform!'";  
          break;
        default:
          commandMessage = "'I'm a programmable ESP8266 that Ryan Sass programmed. Wow, what an engineer he is, eh?'";  
          break;      }        
        publish((char *)commandMessage.c_str());
        break;
    case 5: // sing
        switch(random(3)){
        case 0:
          commandMessage = "'Ok Dave: Daisy...Daisy...'";  
          break;
        case 1:
          commandMessage = "'♩♪I'm on the right track baby, I was born this way!♫'";  
          break;
        default:
          commandMessage = "'I'm a robot, not a singer!'";  
          break;      }          
        publish((char *)commandMessage.c_str());
        break;
    case 6:  //Time Zone: n
      commandMessage = "Time Zone changed to #" + String(timeZone);
      setLocalTimeZone(timeZone);
      publish((char *)commandMessage.c_str());
      rtcMemUpdate();
      break;
    case 7:  //board status
      {
        unsigned long deltaMilliSeconds = (timeAwake - (millis() - awakeStartTime));
        boardStatus(deltaMilliSeconds);
        break;
      } 
    case 8:  //heartbeat command
      heartbeat();
      break;    
    case 9:  //wifi Status
      wifiStatus();
      break;
    case 10:  //[empty command]
      //publish("Blank Command");
      interactCount--;      
      break;    
    case 11:  //Board Restart
      publish("Restarting Board");
      delay(1000);
      restart();
      break;
    case 12:  //howdy
      switch(random(3)){
        case 0:
          commandMessage = "'I'm a robot! Not a cowboy! ;P'";  
          break;
        case 1:
          commandMessage = "'Oh, so you're copying me now?'";  
          break;
        default:
          commandMessage = "'Hey there partner. Hire Ryan Sass.'";  
          break;      }  
      publish((char *)commandMessage.c_str()); 
      break;  
    case 13:
        commandMessage = "'You asked me a question! I'm hoping to eventually link with AI to find simple answers to questions!'";  
        publish((char *)commandMessage.c_str());
        break;
    case 14:
      interactCount--;
    default:  //unknown command
      publish("Unknown Command");
      break;    
  }
  interactCount++;
  client.publish(subscribeTopic, "", true);
  clearChannel = true;
  client.publish(publishInteractCount, (char *)String(interactCount).c_str(), true);
}

int commandCode(String message) {
  if (message == COMMAND_1) return 1;
  if (message == COMMAND_2) return 2;
  if (message == COMMAND_3) return 3;
  if (message == COMMAND_4) return 4;
  if (message == COMMAND_5) return 5;
  if (message == COMMAND_6) return 6;
  if (message == COMMAND_7) return 7;
  if (message == COMMAND_8) return 8;
  if (message == COMMAND_9) return 9;
  if (message == COMMAND_10) return 10;
  if (message == COMMAND_11) return 11;
  if (message == COMMAND_12) return 12;
  if (message == COMMAND_13) return 13;
  if (message == COMMAND_14) return 14;

  return 0;
}


/////////////////////////LED BLINK FUNCTIONS/////////////////////////////////////////////////////////////
void led_ON() {
  digitalWrite(BUILTIN_LED, LOW);
  ledState = true;
}

void led_OFF() {
  digitalWrite(BUILTIN_LED, HIGH);
  ledState = false;
}

void setupCompleteBlink() {
  int count = millis();
  while (millis() <= count + 1500) {
    led_ON();
    delay(14);
    led_OFF();
    delay(41);
  }
  for (int count = 12; count <= 45; count += 4) {
    led_ON();
    delay(count);
    led_OFF();
    delay(2 * count);
  }
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void heartbeatBlink() {
  bool led = ledState;
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  if (firstLoop) {
    led_OFF();
    delay(150);
    led_ON();
    delay(150);
    led_OFF();
    if (ledState) led_ON();
    if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
    return;
  }
  if (led) led_OFF();
  delay(250);
  for (int outerCount = 0; outerCount < 2; outerCount++) {
    for (int innerCount = 0; innerCount < 2; innerCount++) {
      led_ON();
      delay(150);
      led_OFF();
      delay(250);
    }
    delay(500);
  }
  if (led) led_ON();
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void mqttConnFailBlink() {
  bool led = ledState;
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  if (led) led_OFF();
  delay(250);
  for (int count = 0; count < 5; count++) {
    led_ON();
    delay(250);
    led_OFF();
    delay(350);
  }
  delay(2600);
  if (led) led_ON();
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

void acknowledgeCmdBlink() {
  bool led = ledState;
  if (!outputSerial) pinMode(BUILTIN_LED, OUTPUT);
  led_OFF();
  delay(150);
  led_ON();
  delay(150);
  led_OFF();
  delay(250);
  if (led) led_ON();
  if (!outputSerial) pinMode(BUILTIN_LED, INPUT);
}

/////////////////////////PUBLISHING FUNCTIONS ///////////////////////////////////////////////////////////
void pubInteractCount(unsigned long number) {  //send awake in minutes
  snprintf(globalMessageUTC,MSG_BUFFER_SIZE, "%ld", number);
  client.publish(publishInteractCount, globalMessage, true);
}

void publish(char *display, int heartbeatCount) {
  if (heartbeatCount != -1) snprintf(globalMessage, MSG_BUFFER_SIZE, "Heartbeat %ld at %s %s", heartbeatCount, tzname[0], currentTime(true));
  else snprintf(globalMessage, MSG_BUFFER_SIZE, "%s at %s %s", display, tzname[0], currentTime(true));    
  if (outputSerial) debugPrint_aux(globalMessage);
  client.publish(publishTopic, globalMessage);
}

void debugPrint_aux(char *globalMessage) {
  snprintf(globalMessageAlternate, MSG_BUFFER_SIZE, "Publish to [%s]: %s", publishTopic, globalMessage);
  print(globalMessageAlternate);
}

/////////////////////////PRINTING FUNCTIONS /////////////////////////////////////////////////////////////
void print(String toPrint) {
  if (outputSerial) Serial.print(toPrint);
}

void println(String toPrint) {
  if (outputSerial) Serial.println(toPrint);
}

void printSp() {
  println();
  printSpace = false;
}

/////////////////////////STATUS PUBLISHING FUNCTIONS ////////////////////////////////////////////////////
void boardStatus(unsigned long count) {
  String localMessage;
  unsigned long deltaMilliSeconds = (timeAwake - (millis() - awakeStartTime));
  int secs = deltaMilliSeconds / 60000, updatedSleepTime = count / 1000;  
  if (dayTime) publish("It is local DAY Time (8am - 11pm)");
  else if(deepNight) publish("It is VERY late (2am - 5am), go to bed.");
    else publish("It is local NIGHT Time (11pm - 8am)");
  //localMessage = String("Current Local Time (TZ#: ") + String(timeZone) + String("): ") + tzname[0] + " " + currentTime(true);
  //publish((char *)localMessage.c_str());
  print("\n");
}

void wifiStatus() {
  String localMessage = String("Wi-Fi: ") + String(ssid) + String("  || IP: ") + WiFi.localIP().toString();
  publish((char *)localMessage.c_str());
  println();
}

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ SETUP FUNCTION //////////////////////////////////////////////////////////////////////
void setup() {
  checkRTC();           //read memory and see if it is valid
  configure();          //configure board, I/Os and appropriate values
  startWiFi();          //start Wifi connection or start new
  setDateTime();        //set the time via server command
  determineDayNight();  //determine from time zone if it's day or night and set sleep timer appropriately
  setupMQTT();         //connect to mqtt and check for missed messages
  delay(50);
  awakeStartTime = heartbeat();
}  //fires off heartbeat before MQTT connection just to show sign of life

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\ MAIN FUNCTION ///////////////////////////////////////////////////////////////////////
void loop() {
  if (!client.connected()) reconnect();  //reconnect to mqtt if disconnected
  for (int count = 0; count < 10; count++) {         //client loop
    client.loop();
    delay(75);
  }
  if (mqttReconnectAttempts >= 5) restart();  // autosleep after too many attempts
  unsigned long now = millis();
  if(!getInteractCount && (now - originalStartTime) > 3000) {
    client.unsubscribe(publishInteractCount);
    getInteractCount = true; }    \
  if(now > MAX_MILLIS){
    String message = "Gotta reestart! brb ";
    publish((char *)message.c_str());    
    restart();
  }  
  //TIME & Heartbeat CALLS
  if (now - lastMessage > HEARTBEAT) lastMessage = heartbeat();  //heartbeat
} 
   //-----------------------------/END OF CODE/-----------------------------------------------------------------------//