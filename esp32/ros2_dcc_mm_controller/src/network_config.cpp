
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "LittleFS.h"
#include "network_config.h"
#include "tft_printf.h"

#define DEBUG
#ifdef DEBUG
#define DEBUG_PRINT(fmt, ...) \
    do { \
        Serial.printf("DEBUG: %s:%d:%s(): " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)
#else
#define DEBUG_PRINT(fmt, ...) \
    do {} while (0)
#endif

// Create AsyncWebServer object on port 80#include "tft_printf.h"
AsyncWebServer server(80);

// Search for parameter in HTTP POST request
const char* PARAM_INPUT_1 = "ssid";
const char* PARAM_INPUT_2 = "pass";
const char* PARAM_INPUT_3 = "ros_server_ip";

//Variables to save values from HTML form
String ssid;
String pass;
String ros_server_ip;

// File paths to save input values permanently
const char* ssidPath = "/ssid.txt";
const char* passPath = "/pass.txt";
const char* ros_server_ipPath = "/ros_server_ip.txt";

IPAddress localIP;
//IPAddress localIP(192, 168, 1, 200); // hardcoded

// Set your Gateway IP address
IPAddress localGateway;
//IPAddress localGateway(192, 168, 1, 1); //hardcoded
IPAddress subnet(255, 255, 0, 0);

// Timer variables
unsigned long previousMillis = 0;
const long interval = 10000;  // interval to wait for Wi-Fi connection (milliseconds)


// Initialize LittleFS
void initLittleFS() {
  if (!LittleFS.begin(true)) {
    DEBUG_PRINT("An error has occurred while mounting LittleFS\n");
  }
  DEBUG_PRINT("LittleFS mounted successfully\n");
}


// Read File from LittleFS
String readFile(fs::FS &fs, const char * path){
  DEBUG_PRINT("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    DEBUG_PRINT("- failed to open file for reading\n");
    return String();
  }
  
  String fileContent;
  while(file.available()){
    fileContent = file.readStringUntil('\n');
    break;     
  }
  return fileContent;
}

// Write file to LittleFS
void writeFile(fs::FS &fs, const char * path, const char * message){
  DEBUG_PRINT("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    DEBUG_PRINT("- failed to open file for writing\n");
    return;
  }
  if(file.print(message)){
    DEBUG_PRINT("- file written\n");
  } else {
    DEBUG_PRINT("- write failed\n");
  }
}

// Initialize WiFi
bool testWifi() {
  if(ssid=="" || pass==""){
    DEBUG_PRINT("Undefined SSID or IP address\n");
    return false;
  }

  WiFi.mode(WIFI_STA);
#if 0
  localIP.fromString(ip.c_str());
  localGateway.fromString(gateway.c_str());


  if (!WiFi.config(localIP, localGateway, subnet)){
    DEBUG_PRINT("STA Failed to configure\n");
    return false;
  }
#endif
  WiFi.begin(ssid.c_str(), pass.c_str());
  DEBUG_PRINT("Connecting to WiFi...\n");
  tft_printf(ST77XX_MAGENTA, "Connecting\nto WiFi...\n");

  unsigned long currentMillis = millis();
  previousMillis = currentMillis;

  while(WiFi.status() != WL_CONNECTED) {
    currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      DEBUG_PRINT("Failed to connect\n");
      tft_printf(ST77XX_MAGENTA, "Failed to\nconnect to\nWiFi\n");
      return false;
    }
  }

  DEBUG_PRINT("Local IP Address : %s\n", WiFi.localIP());
  return true;
}


NETWORK_CONFIG network_config;

bool configureNetwork(bool forceConfigure, NETWORK_CONFIG *networkConfig) {
  // Serial port for debugging purposes
//  Serial.begin(115200);

  initLittleFS();
 
  // Load values saved in LittleFS
  ssid = readFile(LittleFS, ssidPath);
  pass = readFile(LittleFS, passPath);
  ros_server_ip = readFile(LittleFS, ros_server_ipPath);
  DEBUG_PRINT("ssid : %s\n", ssid.c_str());
  DEBUG_PRINT("pass : %s\n", pass.c_str());
  DEBUG_PRINT("ros_server_ip : %s\n", ros_server_ip.c_str());
  if(testWifi()) {
    //networkConfig->ssid = ssid.c_str();
    //networkConfig->password = pass.c_str();
    //networkConfig.server_ip_address = localIP;
    //networkConfig = network_config;
    return true;
  }
  else {
    // Connect to Wi-Fi network with SSID and password
    WiFi.mode(WIFI_AP);
    const char *ap_name = "Railtrack Wifi";
    DEBUG_PRINT("Setting AP (Access Point)\n");

    // NULL sets an open Access Point
    WiFi.softAP(ap_name, NULL);

    IPAddress IP = WiFi.softAPIP();
    tft_printf(ST77XX_MAGENTA, "Connect to AP:\n%s\nIP: %s\n", ap_name, IP.toString().c_str());
    DEBUG_PRINT("Connect to AP: %s, IP: %s\n", ap_name, IP.toString().c_str());
    
    // Web Server Root URL
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, "/wifimanager.html", "text/html");
    });
    
    server.serveStatic("/", LittleFS, "/");
    
    server.on("/", HTTP_POST, [](AsyncWebServerRequest *request) {
      int params = request->params();
      for(int i=0;i<params;i++){
        const AsyncWebParameter* p = request->getParam(i);
        if(p->isPost()){
          // HTTP POST ssid value
          if (p->name() == PARAM_INPUT_1) {
            ssid = p->value().c_str();
            DEBUG_PRINT("SSID set to: %s\n", ssid );
            // Write file to save value
            writeFile(LittleFS, ssidPath, ssid.c_str());
          }
          // HTTP POST pass value
          if (p->name() == PARAM_INPUT_2) {
            pass = p->value().c_str();
            DEBUG_PRINT("Password set to: %s\n", pass);
            // Write file to save value
            writeFile(LittleFS, passPath, pass.c_str());
          }
          // HTTP POST ip value
          if (p->name() == PARAM_INPUT_3) {
            ros_server_ip = p->value().c_str();
            DEBUG_PRINT("IP Address set to: %s\n", ros_server_ip);
            // Write file to save value
            writeFile(LittleFS, ros_server_ipPath, ros_server_ip.c_str());
          }
          //DEBUG_PRINT("POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
        }
      }
      request->send(200, "text/plain", "Done. Controller will restart");
      tft_printf(ST77XX_MAGENTA, "WiFi confuguration\nstored\nRestarting...\n");
      delay(3000);
      ESP.restart();
    });
    server.begin();
  }
  return false;
}

