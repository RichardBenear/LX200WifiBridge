#include "OledDisplay.h"

#define SCREEN_ADDRESS      0x3C 
#define SCREEN_WIDTH         128 // OLED display width, in pixels
#define SCREEN_HEIGHT         64 // OLED display height, in pixels
#define OLED_RESET            -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define WIFI_HEIGHT           16 // icon size
#define WIFI_WIDTH            16 // icon size

// ============ constants ================
// WiFi ICON, 16x16px
const unsigned char wifi_bmp [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x3f, 0xfc, 0x70, 0x0e, 0xc7, 0xe3, 0x9f, 0xf9, 0x38, 0x1c, 
	0x33, 0xcc, 0x07, 0xe0, 0x0c, 0x30, 0x01, 0x80, 0x01, 0x80, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00
};

// OLED Display object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Initialize the OLED display
void initOledDisplay() {
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    
    display.clearDisplay();
    display.setTextColor(WHITE); // need this
    display.display();
}

void printCentered(Adafruit_SSD1306 &display, const char *text, int y) {
  int16_t textLen = strlen(text);
  int16_t textWidth = textLen * 6;  // 6 pixels per character at size 1
  int16_t x = (128 - textWidth) / 2;
  display.setCursor(x, y);
  display.print(text);
}

// Update the OLED display with current pressure and IP Address
void updateOledDisplay(IPAddress lxStaIpMsg, IPAddress lxApIpMsg, String wdStaIpMsg, IPAddress wdApIpMsg) {
    display.clearDisplay();

    display.drawBitmap(0, 0, wifi_bmp, WIFI_WIDTH, WIFI_HEIGHT, 1);
 
    // OLED display the Pressure
    display.setTextSize(1);

    printCentered(display, "LX200 and", 0);
    printCentered(display, " WiFi Display IP's", 8);

    // Show the LX200 Command Processor IP's
    display.setCursor(0, 20);
    display.print(F("LX-STA:"));
    display.setCursor(40, 20);
    display.print(lxStaIpMsg);

    display.setCursor(0, 32);
    display.print(F("LX-AP :"));
    display.setCursor(40, 32);
    display.print(lxApIpMsg);

    // Show the WiFi Display IP's
    display.setCursor(0, 44);
    display.print(F("WD-STA:"));
    display.setCursor(40, 44);
    display.print(wdStaIpMsg);

    display.setCursor(0, 56);
    display.print(F("WD-AP :"));
    display.setCursor(40, 56);
    display.print(wdApIpMsg);

    


    display.display();
}
