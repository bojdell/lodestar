#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8

// LCD Display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

const int CHAR_WIDTH = 6, CHAR_HEIGHT = 8;          // dimensions of 1 text character in pixels
const uint16_t TEXT_COLOR_DEFAULT = ST7735_WHITE;   // default color used to draw text
const uint16_t TEXT_COLOR_HIGHLIGHT = ST7735_BLACK; // color used to draw highlighted text
const uint16_t BG_COLOR_DEFAULT = ST7735_BLACK;     // default background color for entire display
const uint16_t SPLASH_BG_COLOR_DEFAULT = 0x333C;    // default background color for splash screen
const uint16_t SPLASH_BG_COLOR_HIGHLIGHT = 0xFF00;  // background color used to draw highlighted text on splash screen

const int SPLASH_TIMER = 2000;
int steps = (tft.width() / CHAR_WIDTH) * (tft.height() / CHAR_HEIGHT);

int centerStr(int strLen) {
  return (tft.width() - strLen*CHAR_WIDTH) / 2;
}

//  String str1 = " Project ";
//  String str2 = " Lodestar ";
//  String str3 = "Loading: .....";
//  String DISCLAIMER = "This device only supports straight-line navigation toward destination locations. It will not account for any hazards that may be present along this path. Use this product at your own risk!";

void printSplashScreen() {
  tft.fillScreen(SPLASH_BG_COLOR_DEFAULT);
  tft.setTextColor(TEXT_COLOR_DEFAULT, SPLASH_BG_COLOR_DEFAULT);
  tft.setTextWrap(true);
  tft.setTextSize(3);
  tft.print("CAUTION");
  tft.setTextSize(1);
//  tft.println(DISCLAIMER);
//  tft.setTextSize(2);
//  tft.setCursor(centerStr(str1.length()*2), tft.height() - 6*CHAR_HEIGHT);
//  tft.println(str1);
//  tft.setCursor(centerStr(str2.length()*2), tft.height() - 4*CHAR_HEIGHT);
//  tft.println(str2);
}

void setup()
{
  Serial.begin(9600);
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab
  tft.setTextWrap(true);
  tft.setTextSize(1);
  printSplashScreen();
//  tft.setCursor(0, 0);
}

void loop()
{
//  tft.setTextColor(TEXT_COLOR_DEFAULT, SPLASH_BG_COLOR_DEFAULT);
//  tft.setCursor(centerStr(str3), tft.height() - CHAR_HEIGHT);
//  tft.print("Loading: ");
//  for(int i = 0; i < 5; i++) {
//    delay(SPLASH_TIMER / 5);
//    tft.print(".");
//  }
//  tft.setTextColor(TEXT_COLOR_DEFAULT, SPLASH_BG_COLOR_DEFAULT);
//  tft.setCursor(centerStr(str3), tft.height()/2 - (tft.height()/2)%CHAR_HEIGHT + 2*CHAR_HEIGHT);
//  tft.print("              ");

//  tft.print(".");
//  delay(SPLASH_TIMER / steps);
}
