/***************************************************
 * This is a library for the Adafruit 1.8" SPI display.
 * 
 * This library works with the Adafruit 1.8" TFT Breakout w/SD card
 * ----> http://www.adafruit.com/products/358
 * The 1.8" TFT shield
 * ----> https://www.adafruit.com/product/802
 * The 1.44" TFT breakout
 * ----> https://www.adafruit.com/product/2088
 * as well as Adafruit raw 1.8" TFT display
 * ----> http://www.adafruit.com/products/618
 * 
 * Check out the links above for our tutorials and wiring diagrams
 * These displays use SPI to communicate, 4 or 5 pins are required to
 * interface (RST is optional)
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 * 
 * Written by Limor Fried/Ladyada for Adafruit Industries.
 * Modified by Bodecker DellaMaria and Christopher Buris for ECE 445 @ UIUC Fall 2014
 * Intended for use with Sparkfun Micro Pro 5V (compatible with Arduino Leonardo codebase)
 * MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>

#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303.h>
#include <Adafruit_GPS.h>

#include <LSM303.h>

#include <SoftwareSerial.h>

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield
#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8
#define TFT_SCLK   13
#define TFT_MOSI   11
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST); // display
LSM303 lsm; // mag/accel
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);


#define GPSECHO  false // for debugging

//const float PI = 3.1415926;

const int CHAR_WIDTH = 6, CHAR_HEIGHT = 8;
const int RAD = 56;
int start_x = 2 * CHAR_WIDTH, start_y = CHAR_HEIGHT;
int start_x_offset = 4 * CHAR_WIDTH, start_y_offset = 0;
uint16_t text_color = ST7735_WHITE, bg_color = ST7735_BLACK,
  line_color = 0xFF00;
int center_x = 0, center_y = 0;

#define EARTH_RADIUS 6371000 // mean Earth's radius in km

float getMagDeclination()
{
  return -3.0686/180*PI; // for 61820
}

void setup(void) {
  Serial.begin(115200); // this might need to be bigger to accomodate printing NMEA sentences on serial monitor
  GPS.begin(9600); // initialize hardware serial for GPS

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");

  lsm.init(lsm.device_DLHC, lsm.sa0_high);
  lsm.enableDefault();
  lsm.m_min = (LSM303::vector<int16_t>){  -408,   -585,  -1149};
  lsm.m_max = (LSM303::vector<int16_t>){   508,    370,   173};

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
  center_y = tft.height() / 2 + 22;
  
//  tft.setCursor(start_x, start_y);
//  tft.println("X: ");
//  tft.println("  Y: ");
//  tft.println("  Z: ");

  //send init PMTK packets to GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // PMTK_SET_NMEA_OUTPUT_ALLDATA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
}

int x = 0, y = 0, z = 0;
uint32_t gps_timer = millis(); //timer for GPS display update rates
uint32_t display_timer = millis(); //timer for mag/accel sdisplay update rates
float bearing = 0; // bearing (in degrees) to destination with respect to True North
float display_bearing = 0;
float orientation = 0; //orientation of device with respect to True North (from magnetometer)
float dist_to_dest = 0; //distance to destination

//current coordinates in units of degrees (from uint32_t GPS.latitude_fixed/longitude_fixed)
float curr_lat;
float curr_lon; 
//destination coordinates in units of degrees TODO::SET THESE
float dest_lat = 40.102003; //default to south quad lol
float dest_lon = -88.227170; 

float delta_lat;
float delta_lon;
float heading = 0;

void loop() {
  /*GPS READING*/
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if ((c) && (GPSECHO))
    Serial.write(c); 
    
  /*DO WE WANT TO DO THIS EVERY 5 seconds?*/
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  
  // if millis() or timer wraps around, we'll just reset it
  if (gps_timer > millis())  gps_timer = millis();

  // approximately every 5 seconds or so, print out the current stats
  if (millis() - gps_timer > 5000) { 
    gps_timer = millis(); // reset the timer  
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    tft.setCursor(0, start_y);
    if (!GPS.fix)
    {
      tft.println("No GPS fix :(");
    }
    else {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
      
      //print to display (GPS object has other fields for lat/long in different types)
      tft.print(GPS.latitude); tft.println(GPS.lat); // can tft.print take these args? NO
      tft.print(GPS.longitude); tft.println(GPS.lon);
    }    
    
    
    /*GPS data calculations*/
    //use lat/long fixed ((int)deg*10^7 + microdegrees = deg*10^7) 
    //convert to radians for calculations
    curr_lat = ((float)GPS.latitude_fixed)/10000000/180*PI;
    curr_lon = ((float)GPS.longitude_fixed)/10000000/180*PI;
    
    delta_lat = (dest_lat/180*PI) - curr_lat;
    delta_lon = (dest_lon/180*PI) - curr_lon;
    
    //sign changes
    if (GPS.lat == 'S')
    {
      curr_lat *= -1;
    }
    if (GPS.lon == 'W') //TODO::CHECK CONVENTIONS 
    {
      curr_lon *= -1;    
    }
    //------------------------------------------------
    //TODO::UNCOMMENT, BEARING ALWAYS GOES NORTH!!!!
    //------------------------------------------------
    bearing = 0; //atan2(sin(delta_lon)*cos(dest_lat), cos(curr_lat)*sin(dest_lat) - sin(curr_lat)*cos(dest_lat)*cos(delta_lon));
    //bearing += getMagDeclination();
    
    float a = square(sin(delta_lat)) + cos(curr_lat)*cos(dest_lat)*square(sin(delta_lat));
    float c = 2*atan2(sqrt(a),sqrt(1-a));
    dist_to_dest = c * EARTH_RADIUS;
    Serial.print("distance to destination: "); Serial.println(dist_to_dest);
    tft.print("distance: "); tft.println(dist_to_dest);
  }
  

  // if millis() or timer wraps around, we'll just reset it
  if (display_timer > millis())  display_timer = millis();

  // approximately every <TODO, ~400ms is okay...> seconds or so, print out the current stats
  if (millis() - display_timer > 150) { 
    display_timer = millis(); // reset the timer  \
    
//    lsm.read();
//    heading = lsm.heading();
    
//    tft.setCursor(0, start_y+8*3); //8*3 = 8 pixel text height *3 lines
//    tft.print("  X: "); if(x >= 0) tft.print(" "); if(abs(x) < 100) tft.print(" "); if(abs(x) < 10) tft.print(" "); tft.println(x);
//    tft.print("  Y: "); if(y >= 0) tft.print(" "); if(abs(y) < 100) tft.print(" "); if(abs(y) < 10) tft.print(" "); tft.println(y);
//    tft.print("  Z: "); if(z >= 0) tft.print(" "); if(abs(z) < 100) tft.print(" "); if(abs(z) < 10) tft.print(" "); tft.println(z);

    tft.drawLine(center_x, center_y, center_x + RAD*sin(display_bearing), center_y - RAD*cos(display_bearing), bg_color); // clear old line
    
    display_bearing = bearing - heading; // bearing w/ respect to device orientation
    Serial.print("Relative Bearing: ");
    Serial.println(display_bearing*180/PI);
    
    //might have to reverse/transform direction depending on relative orientations of display to magnetometer
    //Y IS NEGATED BECAUSE HIGHER Y VALUES ARE LOWER ON THE SCREEN!!! NOT CARTESIAN
    tft.drawLine(center_x, center_y, center_x + RAD*sin(display_bearing), center_y - RAD*cos(display_bearing), line_color); //TODO::CHECK sin and cos
  }
//  tft.fillRect(4 * CHAR_WIDTH, CHAR_HEIGHT, 4 * CHAR_WIDTH, 3 * CHAR_HEIGHT, ST7735_RED);
//  tft.drawLine2(center_x, center_y, center_x + x, center_y + y, bg_color, 56);
}


