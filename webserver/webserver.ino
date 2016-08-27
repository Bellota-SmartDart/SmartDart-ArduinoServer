/* 
  FSWebServer - Example WebServer with SPIFFS backend for esp8266
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WebServer library for Arduino environment.
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  upload the contents of the data folder with MkSPIFFS Tool ("ESP8266 Sketch Data Upload" in Tools menu in Arduino IDE)
  or you can upload the contents of a folder if you CD in that folder and run the following command:
  for file in `ls -A1`; do curl -F "file=@$PWD/$file" esp8266fs.local/edit; done
  
  access the sample web page at http://esp8266fs.local
  edit the page by going to http://esp8266fs.local/edit
*/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <FS.h>
#include <Wire.h>

#define FEATURE_PROXIMITY_SENSOR
#define SIMULATE_SENSOR
//#define SIMULATE_WEBSERVER
//#define DEBUG_I2C_DATA
//#define DEBUG_READ_I2C
//#define DEBUG_DISP_INPUT_SW

#define PLAYER_COUNT_MAX 2
#define PLAYER_SET_MAX 3
#define PLAYER_ROUND_MAX 3
#define GAME_MAX_POINT  10
enum {
    LIGHT_OFF = 0,
    LIGHT_ON  = 1
};

typedef enum{
    LED_ACT_GAME_START,
    LED_ACT_ON,
    LED_ACT_OFF,
    LED_ACT_SHOT,
    LED_ACT_MAXPOINT,
    LED_ACT_FINISH,
}circleLedActionType;

// Player structure
struct PLAYER_DATA
{
    struct {
        int setScore[PLAYER_SET_MAX];
    }round[PLAYER_ROUND_MAX];
};

typedef enum {
    GAME_OFF,           // Mirror mode
    GAME_STANDBY,       // User turn switch the dart side from the mirro.
    GAME_RUNNING,       // User start game
    GAME_END,        // The game is finished
}gameStateType;

struct CURRENT_STATUS
{
    int player;
    int round;
    int set;
    gameStateType state;
};

typedef enum {
    SCORE_STANDBY,
    SCORE_NEW
}scoreStateType;

struct SCORE_INPUT
{
    int value;
    scoreStateType state;
};

#define DBG_OUTPUT_PORT Serial

//const char* ssid = "Ethan.Jin Hotspot";
//const char* password = "12345678";
//const char* ssid = "Kims_Clubs";
//const char* password = "11220526";

//const char* ssid = "Campus Community";
//const char* password = "dotheimpossible";

//const char* ssid = "ShowMeYourScore";
//const char* password = "12345678";

const char* ssid = "SmartDartAp";
const char* password = "12345678";
const char* host = "esp8266fs";

ESP8266WebServer server(80);
//holds the current upload
File fsUploadFile;

// GPIO Define
// set pin numbers:
// 0 GPIO ==> It should be zero when download
// 1 GPIO ==> UART 0 Txd
const int gpioBlueLedPin =  2;      // ESP8266 Build In Blue LED
// 3 GPIO ==> UART 0 Rxd
const int gpioI2cSDA = 4;
const int gpioi2cSCK = 5;
// N/A GPIO from 6 to 10
const int gpioProxiIR = 12;
const int gpioCircleLED = 13;
const int interruptI2CPin = 14;
const int gpioWatchPwrCtrl = 15;
const int gpioBtnStart = 16;         // the number of the pushbutton pin

// Blink LED
int ledState = LOW;     
unsigned long previousMillis = 0;
const long interval = 1000;

// I2C Init
#define TOUCH_SLAVE 	0x08
#define I2C_READ_BYTES	16

struct PLAYER_DATA dartPlayer[PLAYER_COUNT_MAX] = {0,};
struct CURRENT_STATUS currStInfo = {0,};
struct SCORE_INPUT currScoreData = {0,};

/*******************************************************************************
* I2C register map
*******************************************************************************/
typedef struct I2CReg
{
#if 0
    /* Read/Writable */
/*0x00*/     uint8 OperationMode;
/*0x01*/     uint8 CommandRegister;
/*0x02*/     uint8 DataRegister;
/*0x03~04*/  uint16 FingerThreshold;
/*0x05~06*/  uint16 NoiseThreshold;
#endif

    /* Read only */
/*0x07*/     uint8 touchPos;
/*0x08~09*/  uint16 EsdCounter;
/*0x0A*/     uint8 FirmwareVersion;
/*0x0B*/     uint8 ModuleVersion;

    /* Read only for inspection mode */
/*0x0C*/     uint8 Cp;
/*0x0D*/    uint8 Idac;
/*0x0E*/    uint8 CompensationIdac;
/*0x0F*/    uint8 modClock;
/*0x10*/    uint8 senseClock;
/*0x11~12*/  uint16 UnfilterRaw;
/*0x13~14*/  uint16 RawData;
/*0x15~16*/  uint16 Baseline;
/*0x17~18*/  uint16 DiffData;
} I2CReg;
struct I2CReg I2CRegisters;

#define OFFSET_OP_MODE			0
#define OFFSET_X_POS			8
#define OFFSET_ESD_COUNTER		9
byte I2cRegister[I2C_READ_BYTES];

long buttonStatus;
long esdCounter;

//format bytes
String formatBytes(size_t bytes){
  if (bytes < 1024){
    return String(bytes)+"B";
  } else if(bytes < (1024 * 1024)){
    return String(bytes/1024.0)+"KB";
  } else if(bytes < (1024 * 1024 * 1024)){
    return String(bytes/1024.0/1024.0)+"MB";
  } else {
    return String(bytes/1024.0/1024.0/1024.0)+"GB";
  }
}

String getContentType(String filename){
  if(server.hasArg("download")) return "application/octet-stream";
  else if(filename.endsWith(".htm")) return "text/html";
  else if(filename.endsWith(".html")) return "text/html";
  else if(filename.endsWith(".css")) return "text/css";
  else if(filename.endsWith(".js")) return "application/javascript";
  else if(filename.endsWith(".png")) return "image/png";
  else if(filename.endsWith(".gif")) return "image/gif";
  else if(filename.endsWith(".jpg")) return "image/jpeg";
  else if(filename.endsWith(".ico")) return "image/x-icon";
  else if(filename.endsWith(".xml")) return "text/xml";
  else if(filename.endsWith(".pdf")) return "application/x-pdf";
  else if(filename.endsWith(".zip")) return "application/x-zip";
  else if(filename.endsWith(".gz")) return "application/x-gzip";
  return "text/plain";
}

bool handleFileRead(String path){
  DBG_OUTPUT_PORT.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.htm";
  String contentType = getContentType(path);
  String pathWithGz = path + ".gz";
  if(SPIFFS.exists(pathWithGz) || SPIFFS.exists(path)){
    if(SPIFFS.exists(pathWithGz))
      path += ".gz";
    File file = SPIFFS.open(path, "r");
    size_t sent = server.streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}

void handleFileUpload(){
  if(server.uri() != "/edit") return;
  HTTPUpload& upload = server.upload();
  if(upload.status == UPLOAD_FILE_START){
    String filename = upload.filename;
    if(!filename.startsWith("/")) filename = "/"+filename;
    DBG_OUTPUT_PORT.print("handleFileUpload Name: "); DBG_OUTPUT_PORT.println(filename);
    fsUploadFile = SPIFFS.open(filename, "w");
    filename = String();
  } else if(upload.status == UPLOAD_FILE_WRITE){
    //DBG_OUTPUT_PORT.print("handleFileUpload Data: "); DBG_OUTPUT_PORT.println(upload.currentSize);
    if(fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if(upload.status == UPLOAD_FILE_END){
    if(fsUploadFile)
      fsUploadFile.close();
    DBG_OUTPUT_PORT.print("handleFileUpload Size: "); DBG_OUTPUT_PORT.println(upload.totalSize);
  }
}

void handleFileDelete(){
  if(server.args() == 0) return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  DBG_OUTPUT_PORT.println("handleFileDelete: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(!SPIFFS.exists(path))
    return server.send(404, "text/plain", "FileNotFound");
  SPIFFS.remove(path);
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileCreate(){
  if(server.args() == 0)
    return server.send(500, "text/plain", "BAD ARGS");
  String path = server.arg(0);
  DBG_OUTPUT_PORT.println("handleFileCreate: " + path);
  if(path == "/")
    return server.send(500, "text/plain", "BAD PATH");
  if(SPIFFS.exists(path))
    return server.send(500, "text/plain", "FILE EXISTS");
  File file = SPIFFS.open(path, "w");
  if(file)
    file.close();
  else
    return server.send(500, "text/plain", "CREATE FAILED");
  server.send(200, "text/plain", "");
  path = String();
}

void handleFileList() {
  if(!server.hasArg("dir")) {server.send(500, "text/plain", "BAD ARGS"); return;}
  
  String path = server.arg("dir");
  DBG_OUTPUT_PORT.println("handleFileList: " + path);
  Dir dir = SPIFFS.openDir(path);
  path = String();

  String output = "[";
  while(dir.next()){
    File entry = dir.openFile("r");
    if (output != "[") output += ',';
    bool isDir = false;
    output += "{\"type\":\"";
    output += (isDir)?"dir":"file";
    output += "\",\"name\":\"";
    output += String(entry.name()).substring(1);
    output += "\"}";
    entry.close();
  }
  
  output += "]";
  server.send(200, "text/json", output);
}

void handleGetBoardStatus(void)
{
#if 1
    String json = "{";
    json += "\"player1\":{";
    json += "\"round1\":["+String(dartPlayer[0].round[0].setScore[0])+","+ String(dartPlayer[0].round[0].setScore[1])+","+String(dartPlayer[0].round[0].setScore[2]) + "],";
    json += "\"round2\":["+String(dartPlayer[0].round[1].setScore[0])+","+ String(dartPlayer[0].round[1].setScore[1])+","+String(dartPlayer[0].round[1].setScore[2]) + "],";
    json += "\"round3\":["+String(dartPlayer[0].round[2].setScore[0])+","+ String(dartPlayer[0].round[2].setScore[1])+","+String(dartPlayer[0].round[2].setScore[2]) + "]";
    json += "},";
    
    json += "\"player2\":{";
    json += "\"round1\":["+String(dartPlayer[1].round[0].setScore[0])+","+String(dartPlayer[1].round[0].setScore[1])+","+String(dartPlayer[1].round[0].setScore[2]) + "],";
    json += "\"round2\":["+String(dartPlayer[1].round[1].setScore[0])+","+String(dartPlayer[1].round[1].setScore[1])+","+String(dartPlayer[1].round[1].setScore[2]) + "],";
    json += "\"round3\":["+String(dartPlayer[1].round[2].setScore[0])+","+String(dartPlayer[1].round[2].setScore[1])+","+String(dartPlayer[1].round[2].setScore[2]) + "]";
    json += "},";

    json += "\"current\":{";
    json += "\"player\":"+String(currStInfo.player+1) + ",";
    json += "\"round\":"+String(currStInfo.round+1) + ",";
    json += "\"set\":"+String(currStInfo.set+1) + ",";
    json += "\"gameStatus\":"+ String((currStInfo.state == GAME_RUNNING) ? 1 : 0);
    json += "}";
    json += "}";
#else
    String json = "{";
    json += "\"heap\":"+String(ESP.getFreeHeap());
    json += ", \"analog\":"+String(analogRead(A0));
    json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    json += "}";
#endif
    DBG_OUTPUT_PORT.printf("%s\n", json.c_str());
    server.send(200, "text/json", json);
    json = String();
}

void initGameData(void)
{
    // Init dartPlayer
    for(int playerCnt = 0; playerCnt < PLAYER_COUNT_MAX; playerCnt++)
    {
        for(int roundCnt = 0; roundCnt < PLAYER_ROUND_MAX; roundCnt++)
        {
            for(int setCnt=0; setCnt < PLAYER_SET_MAX; setCnt++)
            {
                dartPlayer[playerCnt].round[roundCnt].setScore[setCnt] = 0;
            }
        }
    }

    // Init currStInfo
    currStInfo.player = 0;
    currStInfo.round = 0;
    currStInfo.set = 0;
    currStInfo.state = GAME_OFF;

    // Init currScoreData
    currScoreData.value = 0;
    currScoreData.state = SCORE_STANDBY;
}

void handleStart(void)
{
    DBG_OUTPUT_PORT.printf("\n[Web Input] Start Key.");
    initGameData();
    ProcessStartkey();
}

void handleSkip(void)
{
    DBG_OUTPUT_PORT.printf("\n[Web Input] Zero Point.");    
    currScoreData.state = SCORE_NEW;
    currScoreData.value = 0;
}

void setupUart(void)
{
    DBG_OUTPUT_PORT.begin(115200);
    DBG_OUTPUT_PORT.print("\n");
    DBG_OUTPUT_PORT.setDebugOutput(true);
}

void setupSPIFFS(void)
{
    SPIFFS.begin();
    {
        Dir dir = SPIFFS.openDir("/");
        while (dir.next()) {    
            String fileName = dir.fileName();
            size_t fileSize = dir.fileSize();
            DBG_OUTPUT_PORT.printf("FS File: %s, size: %s\n", fileName.c_str(), formatBytes(fileSize).c_str());
        }
        DBG_OUTPUT_PORT.printf("\n");
    }
}

void setupGpio(void)
{
    // initialize the pushbutton pin as an input:
    pinMode(gpioBtnStart, INPUT_PULLUP);
    // initialize the LED pin as an output:
    pinMode(gpioBlueLedPin, OUTPUT);

    pinMode(gpioProxiIR, INPUT_PULLUP);
    pinMode(gpioCircleLED, OUTPUT);
    pinMode(interruptI2CPin, INPUT_PULLUP);
    pinMode(gpioWatchPwrCtrl, OUTPUT);

    // Init OUTPUT
    digitalWrite(gpioBlueLedPin, 0);
    digitalWrite(gpioCircleLED, 0);
    digitalWrite(gpioWatchPwrCtrl, 0); 
    
#ifndef FEATURE_PROXIMITY_SENSOR
    // If the feautre disable, Watch is always ON
    digitalWrite(gpioWatchPwrCtrl,LIGHT_ON);
#endif
}

void setupI2C(void)
{
	pinMode(interruptI2CPin, INPUT_PULLUP);	// touch INT pin
	Wire.begin(gpioI2cSDA, gpioi2cSCK);
}

void setupWiFiWebServer(void)
{
    //WIFI INIT
    DBG_OUTPUT_PORT.printf("Connecting to %s\n", ssid);
    if (String(WiFi.SSID()) != String(ssid)) {
        WiFi.begin(ssid, password);
    }

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DBG_OUTPUT_PORT.print(".");
    }
    DBG_OUTPUT_PORT.println("");
    DBG_OUTPUT_PORT.print("Connected! IP address: ");
    DBG_OUTPUT_PORT.println(WiFi.localIP());

    MDNS.begin(host);
    DBG_OUTPUT_PORT.print("Open http://");
    DBG_OUTPUT_PORT.print(host);
    DBG_OUTPUT_PORT.println(".local/edit to see the file browser");


    //SERVER INIT
    //list directory
    server.on("/list", HTTP_GET, handleFileList);
    //load editor
    server.on("/edit", HTTP_GET, [](){
        if(!handleFileRead("/edit.htm")) server.send(404, "text/plain", "FileNotFound");
    });
    //create file
    server.on("/edit", HTTP_PUT, handleFileCreate);
    //delete file
    server.on("/edit", HTTP_DELETE, handleFileDelete);
    //first callback is called after the request has ended with all parsed arguments
    //second callback handles file uploads at that location
    server.on("/edit", HTTP_POST, [](){ server.send(200, "text/plain", ""); }, handleFileUpload);

    //called when the url is not defined here
    //use it to load content from SPIFFS
    server.onNotFound([](){
    if(!handleFileRead(server.uri()))
      server.send(404, "text/plain", "FileNotFound");
    });

    //get heap status, analog input value and all GPIO statuses in one json call
#if 1
    server.on("/all", HTTP_GET, handleGetBoardStatus);
    server.on("/start", HTTP_GET, handleStart);
    server.on("/skip", HTTP_GET, handleSkip);  
#else
    server.on("/all", HTTP_GET, [](){
    String json = "{";
    json += "\"heap\":"+String(ESP.getFreeHeap());
    json += ", \"analog\":"+String(analogRead(A0));
    json += ", \"gpio\":"+String((uint32_t)(((GPI | GPO) & 0xFFFF) | ((GP16I & 0x01) << 16)));
    json += "}";
    server.send(200, "text/json", json);
    json = String();
    });
#endif
    server.begin();
    DBG_OUTPUT_PORT.println("HTTP server started");    
}

void setup(void)
{
    setupUart();
    setupSPIFFS();
    setupWiFiWebServer();
    setupGpio();
    setupI2C();
    initGameData();
    displayVersion();
}

void blinkLed(void)
{
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;   
        if (ledState == LOW)
          ledState = HIGH;  // Note that this switches the LED *off*
        else
          ledState = LOW;   // Note that this switches the LED *on*
        digitalWrite(LED_BUILTIN, ledState);
    }
}

void i2c_communication()
{
	byte i;
	byte c, d;

	Wire.requestFrom(TOUCH_SLAVE, I2C_READ_BYTES);

	#ifdef DEBUG_READ_I2C
	DBG_OUTPUT_PORT.print("\r\n -->");
	#endif
	for(i = 0; i < I2C_READ_BYTES; i++)
	{
		c = Wire.read();
		I2cRegister[i] = c;

		#ifdef DEBUG_READ_I2C
		DBG_OUTPUT_PORT.printf("%X, ", c);
		#endif
	}
}

#ifdef DEBUG_I2C_DATA
void printI2cRegister()
{
	int i;

	DBG_OUTPUT_PORT.print("\r\n **>");
	for(i = 0; i < I2C_READ_BYTES; i++)
	{
		DBG_OUTPUT_PORT.printf("%X, ", I2cRegister[i]);
	}
}
#endif

#if 0
void playDoremi(uint8 sound)
{
    /* play Doremi */
    noTone(speakerPin);
    delay(5);  // no tone with 5ms delay

    int noteLength = 1000 / noteDuration;
    tone(speakerPin, melody[sound], noteLength);

}
#endif

void sensorRead(void)
{
	byte hbyte, lbyte;
	int timeOut = 0;

	if(digitalRead(interruptI2CPin) == LOW)
    {
		i2c_communication();

		#ifdef DEBUG_I2C_DATA
		printI2cRegister();
		#endif

        currScoreData.value = I2cRegister[0];		
        if(currStInfo.state == GAME_RUNNING)
        {
            currScoreData.state = SCORE_NEW;
        }
        circleLed((currScoreData.value == GAME_MAX_POINT) ? LED_ACT_MAXPOINT : LED_ACT_SHOT);
        DBG_OUTPUT_PORT.printf("\n[%s Status] New Score(%d)\n", (currStInfo.state == GAME_RUNNING) ? "Running" : "Stop", currScoreData.value );

        timeOut = 10000;
        do{
            delay(10);  // TODO: Remove delay and while loop
        }while((digitalRead(interruptI2CPin) == LOW) && --timeOut);

        if(timeOut == 0)
        {
            DBG_OUTPUT_PORT.printf("\n Interrup Pin TimeOut");
        }
	}


#ifdef SIMULATE_SENSOR
    unsigned long time;
    static unsigned long preTime = millis();

    time = millis();
    if(time - preTime > 3000)
    {
        preTime = time;
        if(currStInfo.state == GAME_RUNNING)
        {
            currScoreData.value = random(1,GAME_MAX_POINT);
            currScoreData.state = SCORE_NEW;
        }
    }
#endif
}

void circleLed(circleLedActionType actType)
{
    switch(actType)
    {
        case LED_ACT_GAME_START:
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(1500);
            digitalWrite(gpioCircleLED, LIGHT_OFF);
            delay(1000);
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(1500);
            digitalWrite(gpioCircleLED, LIGHT_OFF);
            delay(1000);
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(1500);
            digitalWrite(gpioCircleLED, LIGHT_OFF);
            delay(1000);
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(3000);
            digitalWrite(gpioCircleLED, LIGHT_OFF);
            break;

        case LED_ACT_ON:
            digitalWrite(gpioCircleLED, LIGHT_ON);        
            break;

        case LED_ACT_OFF:
            digitalWrite(gpioCircleLED, LIGHT_OFF);
            break;

        case LED_ACT_SHOT:
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(100);
            digitalWrite(gpioCircleLED, LIGHT_OFF);
            break;
        case LED_ACT_MAXPOINT:
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(700);
            digitalWrite(gpioCircleLED, LIGHT_OFF);            
            delay(300);
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(700);
            digitalWrite(gpioCircleLED, LIGHT_OFF);            
            delay(300);
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(1000);
            digitalWrite(gpioCircleLED, LIGHT_OFF);            
            delay(500);
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(3000);
            digitalWrite(gpioCircleLED, LIGHT_OFF);            
            break;
            
        case LED_ACT_FINISH:
            digitalWrite(gpioCircleLED, LIGHT_ON);
            delay(4000);
            digitalWrite(gpioCircleLED, LIGHT_OFF);

            break;

        default:
            break;
    }
}

void ProcessStartkey(void)
{
    DBG_OUTPUT_PORT.printf("Start Key Down. ");
    if((currStInfo.state == GAME_STANDBY) || (currStInfo.state == GAME_OFF))
    {
        if(currStInfo.state == GAME_OFF)
        {
            DBG_OUTPUT_PORT.printf("Now it is mirror mode. Let's start game");
        }
        DBG_OUTPUT_PORT.printf("Standby -> Running\n");
        currStInfo.state = GAME_RUNNING;
        circleLed(LED_ACT_GAME_START);
    }
    else if(currStInfo.state == GAME_RUNNING)
    {
        DBG_OUTPUT_PORT.printf("Running -> Standby\n");
        currStInfo.state = GAME_STANDBY;

    }
    else if(currStInfo.state == GAME_END)
    {
        initGameData();
        currStInfo.state = GAME_RUNNING;
        circleLed(LED_ACT_FINISH);

        DBG_OUTPUT_PORT.printf("Game End -> Running\n");
    }
}

void keyInOutCheck(void)
{
    int  btnStartState = 0;
    int  btnProxState = 0;

    static int prevBtnProxState = -1;
    static int prevBtnStartState = -1;
    
    // Start Button Check
     btnStartState = digitalRead(gpioBtnStart);
     if(btnStartState != prevBtnStartState)
     {
        prevBtnStartState = btnStartState;
        if(!btnStartState)
        {
            prevBtnStartState = btnStartState;
            ProcessStartkey();
        }
    }

#ifdef FEATURE_PROXIMITY_SENSOR
    // Proximity IR Check
    btnProxState = digitalRead(gpioProxiIR);
    if(btnProxState != prevBtnProxState)
    {
        prevBtnProxState = btnProxState;
        DBG_OUTPUT_PORT.printf("Proximity Sensor %s", (btnProxState) ? "ON" : "OFF");
        if(btnProxState)
        {
            currStInfo.state == GAME_OFF;
            // Watch ON
            digitalWrite(gpioWatchPwrCtrl, LIGHT_ON);            
            DBG_OUTPUT_PORT.printf("  Watch ON\n");
        }
        else
        {
            currStInfo.state == GAME_RUNNING;
            // Watch OFF
            digitalWrite(gpioWatchPwrCtrl, LIGHT_OFF);
            DBG_OUTPUT_PORT.printf("  Watch OFF\n");

            // CicleLED ON
            circleLed(LED_ACT_GAME_START);
        }
    }
#endif
}

void updateScore(void)
{
    if((currStInfo.state == GAME_RUNNING) && (currScoreData.state == SCORE_NEW))
    {
        currScoreData.state = SCORE_STANDBY;
        DBG_OUTPUT_PORT.printf("Player(%d),Round(%d),Step(%d), NewScore(%d)\n", currStInfo.player, currStInfo.round, currStInfo.set, currScoreData.value);
        dartPlayer[currStInfo.player].round[currStInfo.round].setScore[currStInfo.set] = currScoreData.value;
        if(++currStInfo.set >= PLAYER_SET_MAX)
        {
            currStInfo.set = 0;
            if(++currStInfo.player >= PLAYER_COUNT_MAX)
            {
                currStInfo.player = 0;
                if(++currStInfo.round >= PLAYER_ROUND_MAX)
                {
                    currStInfo.round = 0;
                    currStInfo.state = GAME_END;
                }
            }
        }
    }
}

#ifdef SIMULATE_WEBSERVER
void simulateWebUpload(void)
{
    unsigned long time;
    static unsigned long preTime = millis();

    time = millis();
    if(time - preTime > 1000)
    {
        preTime = time;
        handleGetBoardStatus();

    }
}
#endif

#ifdef DEBUG_DISP_INPUT_SW
void debugDispInputSw(void)
{
    unsigned long time;
    static unsigned long preTime = millis();

    time = millis();
    if(time - preTime > 1000)
    {
        preTime = time;
        DBG_OUTPUT_PORT.printf("Start SW(%d), ProxSW(%d)\n", digitalRead(gpioBtnStart), digitalRead(gpioProxiIR));
    }
}
#endif

void displayVersion(void)
{
    int version = 0;
    // v1 : 
    
    version = 1;    // 2016.08.28 added simulator
    
    DBG_OUTPUT_PORT.printf("F/W Ver(%d), Date(%s), Time(%s)", version,__DATE__, __TIME__);
}

void loop(void){
    unsigned long time;
    static unsigned long preTime = millis();
    
    server.handleClient();
    blinkLed(); 
    keyInOutCheck();
    sensorRead();
    updateScore();

#ifdef SIMULATE_WEBSERVER
    simulateWebUpload();
#endif

#ifdef DEBUG_DISP_INPUT_SW
    debugDispInputSw();    
#endif

#if 0
    time = millis();
    if(time - preTime > 3000)
    {
        preTime = time;
        DBG_OUTPUT_PORT.printf(".");
    }
#endif
}
