// ========================================
// ======== LX200 Command Processor =======
// ========================================
// Author: Richard Benear 5/25
//
// Runs on a SEEED XIAO ESP32-C3 processor with external antenna.
// Adds SkySafari and Stellarium capability to the DDScopeX project.
//
// Connects via wifi to Sky Safari Plus/Pro or Stellarium Mobile clien.
//    Sends the LX200 protocol commands to Teensy Serial8 via ESP32C3 Serial1. 
//    Teensy responds with data which is relayed to Wifi Client Sky Safari
//    or Stellarium Mobile.
//
//  Note: Originally, I tried integrating this code with the WiFi Hand
//    Controller (screen mirror) code that runs on the ESP32-S3 and is
//    connected to Teensy via USB. Multiple tasks running at different rates
//    required extensive use of semaphores and additional complexity for
//    debugging since the USB Serial was shared. Performance was not very
//    good either, so this separate ESP32-C3 implementation is cleaner.
//

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiServer.h>
#include <Wire.h>
#include "OledDisplay.h"
#include "esp_wifi.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "../include/secrets.h"

#define LX200_AP_SSID             "LX200-ESP32"
#define LX200_AP_PASSWORD            "password"
#define LX200_AP_IP_ADDR          {192,168,4,1} 
#define LX200_AP_GW_ADDR          {192,168,4,1} 
#define WIFI_DISPLAY_AP_IP_ADDR   {192,168,4,2} 

#define SERIAL_TEENSY Serial1
#define SERIAL_DEBUG Serial

#define I2C_SDA D4 
#define I2C_SCL D5 
#define RESET_PIN D10 

#define TEENSY_ACK_TIMEOUT 500

WiFiServer lx200Server(4030);

static bool receivingLX200 = false;
volatile bool clientConnected = false;
static int hashCount = 0;

// Check for LX200 commands that need no response to client
bool isNoResponseCommand(const String &cmd) {
  return (
    cmd == ":Me#"  ||  // Start moving East
    cmd == ":Mn#"  ||  // Start moving North
    cmd == ":Ms#"  ||  // Start moving South
    cmd == ":Mw#"  ||  // Start moving West
    cmd == ":Qe#"  ||  // Abort slew East
    cmd == ":Qn#"  ||  // Abort slew North
    cmd == ":Qs#"  ||  // Abort slew South
    cmd == ":Qw#"  ||  // Abort slew West
    cmd == ":RC#"  ||  // Set slew rate to centering
    cmd == ":RF#"  ||  // Set slew rate to fast
    cmd == ":RG#"  ||  // Set slew rate to guiding
    cmd == ":RM#"  ||  // Set slew rate to find
    cmd == ":RS#"  ||  // Set slew rate to max, or Sync for LX200 classic
    cmd == ":W1#"  ||  // Set site 1
    cmd == ":CS#"      // Synchronize the telescope with current RA/DEC
  );
}

// Check for LX200 commands that are Specific to this App
String checkForAppSpecificCmds(const String &cmd) {
  if (cmd == ":GVP#")  return "OnStepX.DDScopeX#"; // Product Name
  if (cmd == ":GVN#")  return "2.0#";        // Firmware Version
  if (cmd == ":GVD#")  return "May 2025#";   // Firmware Date
  if (cmd == ":GVT#")  return "08:02:00#";   // Telescope Firmware time
  //if (cmd == ":D#")    return "#";         // Requests a string of bars indicating the distance to the current target location
  //if (cmd == ":CM#")   return "Syncd Object#"; 
  //if (cmd == ":GW#")   return "AN1#";      // Get Scope alignment status <mount><tracking><alignment>
                                             //   mount: A-AzEl mounted, P-Equatorially mounted, G-german mounted equatorial
                                             //   tracking: T-tracking, N-not tracking
                                             //   alignment: 0-needs alignment, 1-one star aligned, 2-two star aligned, 3-three star aligned
  return "";
}

// :MS#   returns:
    //              0=Goto is possible
    //              1=below the horizon limit
    //              2=above overhead limit
    //              3=controller in standby
    //              4=mount is parked
    //              5=Goto in progress
    //              6=outside limits (AXIS2_LIMIT_MAX, AXIS2_LIMIT_MIN, AXIS1_LIMIT_MIN/MAX, MERIDIAN_E/W)
    //              7=hardware fault
    //              8=already in motion
    //              9=unspecified error

const char* getAsciiLabel(uint8_t c) {
  static char label[4];  // must be static to return a valid pointer

  switch (c) {
    case '\r': return "\\r";
    case '\n': return "\\n";
    case '\t': return "\\t";
    default:
      if (isprint(c)) {
        label[0] = (char)c;
        label[1] = '\0';
        return label;
      } else {
        return ".";
      }
  }
}

// ================ Handshake Teensy =====================
// Handshake Teensy: Send 'L' and wait for 'K'
void handshakeTeensy() {
  SERIAL_TEENSY.write('L');
  SERIAL_TEENSY.flush();

  unsigned long ackStart = millis();
  while ((millis() - ackStart) < TEENSY_ACK_TIMEOUT) {
    if (SERIAL_TEENSY.available() && SERIAL_TEENSY.read() == 'K') {
      delay(3);
      // Flush any remaining pre-response garbage
      while (SERIAL_TEENSY.available()) SERIAL_TEENSY.read();
      break;
    }
  }
}

// ============= Read Teensy Response =====================
String readTeensyResponse() {
  String tResponse = "";
  unsigned long startWait = millis();

  // Wait for at least 1 byte
  while ((millis() - startWait) < 2300) {
    if (SERIAL_TEENSY.available()) break;
  }

  if (!SERIAL_TEENSY.available()) {
    SERIAL_DEBUG.println("Timeout waiting for response ':'");
    return "";  // Return minimal terminator to avoid client crash
  }

  // Read until '#' is received or timeout
  unsigned long readStart = millis();
  while ((millis() - readStart) < 350) {
    while (SERIAL_TEENSY.available()) {
      char rc = SERIAL_TEENSY.read();

      // Skip early junk like stray 'K', '\n', etc.
      if (rc == 'K' || rc == '\n' || rc == '\r') continue;

      tResponse += rc;
      if (rc == '#') {
        return tResponse;
      }
    }
  }

  SERIAL_DEBUG.println("Timeout waiting for Teensy response '#'");
  return tResponse;  // Might be partial
}

/// =============Process LX200 Command =====================
// Process the LX200 incoming command and determine if it needs to be
//    fetched from Teensy, no return, or return a special string from here.
String processLX200Command(const String &cmd) {

  String localResp = checkForAppSpecificCmds(cmd);
  if (localResp.length() != 0) return localResp;

  String truncCmd = cmd;  // make a mutable copy

  //Handle Specific: SkySafari is sending an unsupported format for timezone in OnStep
  //so truncate the decimal
  if (cmd == ":SG+06.0#") {
    int dotIndex = truncCmd.indexOf('.');
    int hashIndex = truncCmd.indexOf('#');

    if (dotIndex != -1 && hashIndex != -1 && dotIndex < hashIndex) {
      truncCmd = truncCmd.substring(0, dotIndex) + truncCmd.substring(hashIndex);
    }
  }

  //SERIAL_DEBUG.print("truncCmd="); SERIAL_DEBUG.println(truncCmd);
  handshakeTeensy();
  SERIAL_TEENSY.print(truncCmd);
  SERIAL_TEENSY.flush();
  return readTeensyResponse();
}

// ============== Handle LX200 CLient =====================
void handleLX200Client() {
  WiFiClient client = lx200Server.available();

  if (!client || !client.connected()) return;
  client.setNoDelay(true);  // <-- important
  String lx200Cmd = "";
  bool receivingCmd = false;
  unsigned long start = millis();

  // Not sure of the exact minimum for timeout but 10 sec works all the time
  while (client.connected()) {
    if (millis() - start > 10000) {
      SERIAL_DEBUG.println("[LX200] Client timeout.");
      break;
    }
    while (client.available()) {
      start = millis();  // Reset timeout on each byte
      char c = client.read();
      //Serial.printf("Received from client, byte: 0x%02X (%s)\n", (uint8_t)c, getAsciiLabel((uint8_t)c));

      // Stellarium Mobile sends 0x06 to check for LX200 mount type
      if (c == 0x06) {
        client.print('A');
        client.flush();
        SERIAL_DEBUG.println("Sent 'A'");
        continue;
      }

      // Wait for ':' to begin a new command, Stellarium mobile puts a '#' in front of ':' many times
      if (!receivingCmd && c == ':') {
        receivingCmd = true;
        lx200Cmd = ":";
        continue;
      }

      lx200Cmd += c;

      if (c == '#' && receivingCmd) {
        receivingCmd = false;

        String response = processLX200Command(lx200Cmd);

         // Remove hash from bool responses
        if (response == "1#" || response == "0#") {
          response = response.substring(0, 1);
        }

        // must use break; here and not return; or SkySafari disconnects
        // because of the :RS# command, which returns nothing, is immediately
        // followed by the client sending another command and the while loop must         
        // get ready for next command quickly or misses the command. (e.g. :GD#)
        if (isNoResponseCommand(lx200Cmd)) {
          //SERIAL_DEBUG.printf("Skipping response for: %s\n", lx200Cmd.c_str());
          break;
        }

        if (response.length() > 0) {
          // Stellarium wants this string and not the OnStep reply of "1#"
          // So the :SC command was sent to OnStep but here we return this string instead.
          if (lx200Cmd.startsWith(":SC")) {
            response = "1Updating Planetary Data#          #";
          }

          // You MUST return a '1' ('#' get's stripped later) for Stellarium GOTO
          // OnStepX returns nothing, just a '#'.
          if (lx200Cmd == ":Q#") {
            response = "1";
          }

          client.write((const uint8_t *)response.c_str(), response.length());
          client.flush();
          SERIAL_DEBUG.printf("CmdFromClient: %-13s  RespToClient: %s\n", lx200Cmd.c_str(), response.c_str());
        }
      }
    }
    yield();  // helps WiFi stack
  }
}

// =================== SETUP =====================
void setup() {
  
  delay(5000);  // this here to allow time to get the debug terminal going

  SERIAL_DEBUG.begin(115200);
  SERIAL_DEBUG.println("Debug port started");

  // SERIAL_TEENSY.begin(460800, SERIAL_8N1, D7, D6);
  SERIAL_TEENSY.begin(230400, SERIAL_8N1, D7, D6); //D7=RX, D6=TX

  pinMode(RESET_PIN, INPUT_PULLUP);

  delay(5);
  // Disable brownout detector on the WeMos ESP32 D1 Mini if that is hardware used
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.println("Starting TEENSY serial...");
  delay(100);

  while (SERIAL_TEENSY.available()) SERIAL_TEENSY.read();  // Flush junk

  // Initialize I2C on the ESP32-C3's default pins
  initOledDisplay();

  // Using Dual mode Wifi
  WiFi.mode(WIFI_AP_STA);

  // Start Station Mode WiFi
  WiFi.begin(LX200_STA_SSID, LX200_STA_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SERIAL_DEBUG.print(".");
  }

  IPAddress lxStaIpMsg = WiFi.localIP();
  SERIAL_DEBUG.print("\nSTA IP Address: ");
  SERIAL_DEBUG.println(lxStaIpMsg);

  delay(10);

  WiFi.setSleep(false);  // Prevent disconnects

  // Set static AP IP
  WiFi.softAPConfig(
    IPAddress(LX200_AP_IP_ADDR), 
    IPAddress(LX200_AP_GW_ADDR), 
    IPAddress(255,255,255,0));

  // Now start AP :  Channel 1, hidden SSID off, max 1 client
  bool apStarted = WiFi.softAP(LX200_AP_SSID, LX200_AP_PASSWORD, 1, 0, 1);
  if (!apStarted) {
    SERIAL_DEBUG.println("Failed to start Access Point!");
  } else {
    SERIAL_DEBUG.println("Access Point started");
  }

  // Print AP IP address
  IPAddress lxApIpMsg = WiFi.softAPIP();
  SERIAL_DEBUG.print("AP IP Address: ");
  SERIAL_DEBUG.println(lxApIpMsg);

  // Start TCP server
  lx200Server.begin();
  SERIAL_DEBUG.println("LX200 TCP Server started on port 4030");
  Serial.printf("WiFi RSSI: %d dBm\n", WiFi.RSSI());

  WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Max power 
}

// ====================== LOOP =======================
unsigned long lastWifiIpCheck = 0;
bool wifiIpReceived = false;
IPAddress wdStaIp;

void loop() {
  handleLX200Client();
  yield();

  // Check for the IP Address of the Wifi Display ESP32 and display it on the OLED
  if (!wifiIpReceived && millis() - lastWifiIpCheck >= 5000) {
    lastWifiIpCheck = millis();
    
    String wdStaIpMsg = processLX200Command(":GI#");
    Serial.print("wdStaIpMsg = "); Serial.println(wdStaIpMsg);

    // Only proceed if it is long enough and ends with '#'
    if (wdStaIpMsg.length() > 1 && wdStaIpMsg.endsWith("#")) {
      wdStaIpMsg.remove(wdStaIpMsg.length() - 1); // remove trailing '#'
      wdStaIpMsg.trim();                          // remove newline/whitespace

      IPAddress lxStaIpMsg = WiFi.localIP();
      updateOledDisplay(lxStaIpMsg, LX200_AP_IP_ADDR, wdStaIpMsg, WIFI_DISPLAY_AP_IP_ADDR);
      wifiIpReceived = true;  // Uncomment if you want to stop polling
      Serial.print("got the IP Address from Teensy");
    }
  }

  // Software generated Reset from Teensy
  if (digitalRead(RESET_PIN) == LOW) {
    SERIAL_DEBUG.println("Reset requested from Teensy");
    esp_restart();
  }
}



