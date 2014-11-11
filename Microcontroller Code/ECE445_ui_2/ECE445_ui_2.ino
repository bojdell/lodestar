/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  Modified by Bodecker DellaMaria and Christopher Buris for ECE 445 @ UIUC Fall 2014
  Intended for use with Sparkfun Micro Pro 5V (compatible with Arduino Leonardo codebase)
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303.h>

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     8
#define TFT_RST    9
#define TFT_DC     10
#define TFT_SCLK   15
#define TFT_MOSI   16
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_LSM303 lsm;

//const float PI = 3.1415926;

const int CHAR_WIDTH = 6, CHAR_HEIGHT = 8;
const int RAD = 56;
int start_x = 2 * CHAR_WIDTH, start_y = CHAR_HEIGHT;
int start_x_offset = 4 * CHAR_WIDTH, start_y_offset = 0;
uint16_t text_color = ST7735_WHITE, bg_color = ST7735_BLACK,
  line_color = 0;
int center_x = 0, center_y = 0;

void setup(void) {
  Serial.begin(9600);

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");

  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }

  uint16_t time = millis();
  tft.fillScreen(ST7735_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text
  tft.fillScreen(ST7735_BLACK);
  delay(1000);
  
  tft.setTextColor(text_color, bg_color);
  tft.setTextWrap(true);
  
  center_x = tft.width() / 2;
  center_y = tft.height() / 2;
  
//  tft.setCursor(start_x, start_y);
//  tft.println("X: ");
//  tft.println("  Y: ");
//  tft.println("  Z: ");
}

int x = 0, y = 0, z = 0;
void loop() {
  lsm.read();
  x = (int)lsm.magData.x;
  y = (int)lsm.magData.y;
  z = (int)lsm.magData.z;
  
//  tft.setTextColor(text_color);
//  tft.setCursor(start_x + start_x_offset, start_y + start_y_offset);
//  tft.println(x);
//  tft.setCursor(start_x + start_x_offset, start_y + start_y_offset + CHAR_HEIGHT);
//  tft.println(y);
//  tft.setCursor(start_x + start_x_offset, start_y + start_y_offset + 2 * CHAR_HEIGHT);
//  tft.println(z);

//  tft.setTextColor(text_anticolor);
//  tft.setCursor(start_x + start_x_offset, start_y + start_y_offset);
//  tft.println(x);
//  tft.setCursor(start_x + start_x_offset, start_y + start_y_offset + CHAR_HEIGHT);
//  tft.println(y);
//  tft.setCursor(start_x + start_x_offset, start_y + start_y_offset + 2 * CHAR_HEIGHT);
//  tft.println(z);

  tft.setCursor(0, start_y);
  tft.print("  X: "); if(x > 0) tft.print(" "); tft.println(x);
  tft.print("  Y: "); if(y > 0) tft.print(" "); tft.println(y);
  tft.print("  Z: "); if(z > 0) tft.print(" "); tft.println(z);
  
  tft.drawLine2(center_x, center_y, center_x + x, center_y + y, line_color, 56);
  line_color += 10;
  if(line_color > 0xFFFF)
    line_color = 0;
  // Calculate the angle of the vector y,x in radians
  float heading = atan2(y,x);
  
  // Normalize to 0-2PI
  if (heading < 0)
  {
    heading = 2*PI + heading;
  }
  Serial.print("Compass Heading: ");
  Serial.println(heading);
  
  delay(200);
  //tft.drawLine(center_x, center_y, center_x + RAD*sin(heading), center_y + RAD*cos(heading), line_color);
//  tft.fillRect(4 * CHAR_WIDTH, CHAR_HEIGHT, 4 * CHAR_WIDTH, 3 * CHAR_HEIGHT, ST7735_RED);
//  tft.drawLine2(center_x, center_y, center_x + x, center_y + y, bg_color, 56);
}
