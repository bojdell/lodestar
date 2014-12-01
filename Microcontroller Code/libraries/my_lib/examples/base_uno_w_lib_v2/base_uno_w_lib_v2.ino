/****************************************
 * ECE 445 - UIUC Senior Design Fall 2014
 * GPS Compass Device
 * Bodecker DellaMaria
 * Chris Buris
 *
 * MICROCONTROLLER CODE
 ****************************************/

/****************************************
 * LIBRARIES
 ****************************************/
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_LSM303.h>
#include <Adafruit_GPS.h>
#include <my_lib.h>
#include <SoftwareSerial.h>
#include <avr/eeprom.h>

/****************************************
 * PINS
 ****************************************/
#define TFT_CS     10
#define TFT_RST    9
#define TFT_DC     8
#define BUTTON_1   7
#define BUTTON_2   6
#define BUTTON_3   5
#define BUTTON_4   4

/****************************************
 * SENSOR INIT
 ****************************************/
// LCD Display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Magnetometer & Accelerometer
Adafruit_LSM303 lsm;

// GPS
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false // for debugging

/****************************************
 * GLOBAL VARS & CONSTANTS
 ****************************************/
// LCD Display
const int CHAR_WIDTH = 6, CHAR_HEIGHT = 8;          // dimensions of 1 text character in pixels
const uint16_t LINE_COLOR_DEFAULT = 0xFF00;         // default color used to draw arrow lines
const uint16_t LINE_COLOR_SUCCESS = ST7735_GREEN;   // color used to draw arrow lines when navigating correctly
const uint16_t TEXT_COLOR_DEFAULT = ST7735_WHITE;   // default color used to draw text
const uint16_t TEXT_COLOR_HIGHLIGHT = ST7735_BLACK; // color used to draw highlighted text
const uint16_t BG_COLOR_DEFAULT = ST7735_BLACK;     // default background color for entire display
const uint16_t BG_COLOR_HIGHLIGHT = 0xFF00;         // background color used to draw highlighted text

uint16_t line_color = LINE_COLOR_DEFAULT;           // initialize arrow lines to be default color
int center_x = tft.width() / 2;                     // init center_x and center_y (center of navigation circle)
int center_y = tft.height() / 2 + 22;                 // ^^

// Arrow
const int ARROW_RAD = 46;               // radius of outermost arrow point
const int ARROW_RAD2 = ARROW_RAD - 6;   // radius of L & R arrow tips
const float ARROW_RAD2_ANGLE = PI / 10; // angle to arrow tips in radians

// Center Circle / Nav Guidelines
const int CIRCLE_RAD = 30;              // radius of center circle (if present)
const int GUIDELINE_ANGLE = 10;         // angle in deg of nav guidelines

// Locations
const int NUM_LOCS = 3;                 // number of locations we have stored
float locs[NUM_LOCS][2];                // data structure to hold stored locations. data persists in EEPROM and is read in during init()
int curr_loc = 0;                       // the location we are currently navigating to

// coordinates for drawing location
const int LOC_START_X = 2 * CHAR_WIDTH;
const int LOC_START_Y = tft.height() - CHAR_HEIGHT;

// Device Modes
const uint8_t NORMAL_MODE = 0;      // device operates "normally", i.e. it navigates to currently selected location
const uint8_t COMPASS_MODE = 1;     // device operates as a compass, and the navigation arrow points North
const uint8_t DEBUG_MODE = 2;       // device operates in debug mode, displaying extra sensor and state data to display
uint8_t device_mode = COMPASS_MODE; // initialize device mode

// coordinates for drawing device mode
const int MODE_START_X = 0;
const int MODE_START_Y = 4 * CHAR_HEIGHT;

// Buttons
const int BUTTON_PRESS_LONG = 600;   // milliseconds until a long button press is registered (used to set current location)  

// Misc.
#define EARTH_RADIUS 6371000        // mean Earth's radius in km

/****************************************
 * BEGIN FUNCTIONS
 ****************************************/

// returns the magnetic declination for our current location
float getMagDeclination()
{
  return -3.0686/180*PI;            // for 61820 zip code
}

// TODO:
// 1) hold to set location
// 2) sensor fusion
// 3) "debug mode"

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

void updateLocText(int loc) {
  tft.setCursor(LOC_START_X, LOC_START_Y);
  tft.print("Loc: ");
  for(int i = 0; i < NUM_LOCS; i++) {
    if(i == loc) {
      tft.setTextColor(TEXT_COLOR_HIGHLIGHT, BG_COLOR_HIGHLIGHT); tft.print(i+1);
      tft.setTextColor(TEXT_COLOR_DEFAULT, BG_COLOR_DEFAULT); tft.print(" ");
    }
    else {
      tft.print(i+1); tft.print(" ");
    }
  }
  return;
}

void setup(void) {
  Serial.begin(115200); // this might need to be bigger to accomodate printing NMEA sentences on serial monitor
  GPS.begin(9600); // initialize hardware serial for GPS
  
  //send init PMTK packets to GPS
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // PMTK_SET_NMEA_OUTPUT_ALLDATA
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  
  eeprom_read_block((void*)&locs, (void*)0, 2*NUM_LOCS*sizeof(float));

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");
  
  //push buttons
  pinMode(BUTTON_1, INPUT); 
  pinMode(BUTTON_2, INPUT);
  pinMode(BUTTON_3, INPUT); 
  pinMode(BUTTON_4, INPUT); 
  
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    while (1);
  }
  lsm.write8(LSM303_ADDRESS_MAG, lsm.LSM303_REGISTER_MAG_CRA_REG_M, (byte)0x90); // change output rate to 15Hz

  uint16_t time = millis();
  tft.fillScreen(BG_COLOR_DEFAULT);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);
  
  tft.setTextColor(TEXT_COLOR_DEFAULT, BG_COLOR_DEFAULT);
  tft.setTextWrap(true);
  
//  tft.drawCircle(center_x, center_y, CENTER_RAD, TEXT_COLOR_DEFAULT);
  
  tft.drawLine(center_x, center_y, center_x + ARROW_RAD2*sin( ((float)GUIDELINE_ANGLE)/180 * PI ), center_y - ARROW_RAD2*cos( ((float)GUIDELINE_ANGLE)/180 * PI ), TEXT_COLOR_DEFAULT);
  tft.drawLine(center_x, center_y, center_x + ARROW_RAD2*sin( -1 * ((float)GUIDELINE_ANGLE)/180 * PI ), center_y - ARROW_RAD2*cos( -1 * ((float)GUIDELINE_ANGLE)/180 * PI ), TEXT_COLOR_DEFAULT);
  
  updateLocText(curr_loc);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

int x = 0, y = 0, z = 0;
uint32_t gps_timer = millis(); //timer for GPS display update rates
uint32_t display_timer = millis(); //timer for mag/accel sdisplay update rates
uint32_t loc_reset_timer = millis(); // timer for long button press
float bearing = 0; // bearing (in degrees) to destination with respect to True North
float display_bearing = 0;
float orientation = 0; //orientation of device with respect to True North (from magnetometer)
float smoothed_orientation = 0; // smoothed orientation
float filterVal = .5; // filter value: 0 = no smoothing, 1 = no update (filters everything!)
float dist_to_dest = 0; //distance to destination

//current coordinates in units of degrees (from uint32_t GPS.latitude_fixed/longitude_fixed)
float curr_lat;
float curr_lon; 
//destination coordinates in units of degrees TODO::SET THESE
float dest_lat = 40.102003/180*PI; //default to south quad lol
float dest_lon = -88.227170/180*PI; 

float delta_lat;
float delta_lon;

boolean button3 = false;

int16_t mag_min[3] = { -385,  -638, -1228 }; //{ -408,  -585,  -1149 };
int16_t mag_max[3] = {  601,   448,   372 }; //{  508,   370,   173  };

boolean fake_fix = false;

void loop() {
  /*GPS READING*/
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
    
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
    
//    Serial.print("\nTime: ");
//    Serial.print(GPS.hour, DEC); Serial.print(':');
//    Serial.print(GPS.minute, DEC); Serial.print(':');
//    Serial.print(GPS.seconds, DEC); Serial.print('.');
//    Serial.println(GPS.milliseconds);
//    Serial.print("Date: ");
//    Serial.print(GPS.day, DEC); Serial.print('/');
//    Serial.print(GPS.month, DEC); Serial.print("/20");
//    Serial.println(GPS.year, DEC);
//    Serial.print("Fix: "); Serial.print((int)GPS.fix);
//    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    tft.setCursor(0, CHAR_HEIGHT);
    if (!GPS.fix)
    {
      tft.println("No fix");
    }
    else {
      tft.println("Ya fix");
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
      
      //use lat/long fixed ((int)deg*10^7 + microdegrees = deg*10^7) 
      //convert to radians for calculations
      curr_lat = ((float)GPS.latitude_fixed)/10000000/180*PI;
      curr_lon = ((float)GPS.longitude_fixed)/10000000/180*PI;
      
      //sign changes
      if (GPS.lat == 'S')
      {
        curr_lat *= -1;
      }
      if (GPS.lon == 'W') //TODO::CHECK CONVENTIONS 
      {
        curr_lon *= -1;    
      }
    }
    
    /*GPS data calculations*/
    
    delta_lat = dest_lat - curr_lat;
    delta_lon = dest_lon - curr_lon;

    bearing = atan2(sin(delta_lon)*cos(dest_lat), cos(curr_lat)*sin(dest_lat) - sin(curr_lat)*cos(dest_lat)*cos(delta_lon));
    
    float a = square(sin(delta_lat / 2)) + cos(curr_lat)*cos(dest_lat)*square(sin(delta_lon / 2));
    float c = 2*atan2(sqrt(a),sqrt(1-a));
    dist_to_dest = c * EARTH_RADIUS;
//    Serial.print("distance to destination: "); Serial.println((int)dist_to_dest);
    tft.print("Dist: "); tft.print((int)dist_to_dest); tft.println(" m");
  }
  
//  Serial.print("curr_lat: "); Serial.println(curr_lat*180/PI, 4);
//  Serial.print("curr_lon: "); Serial.println(curr_lon*180/PI, 4);
//  Serial.print("bearing: "); Serial.println(bearing*180/PI);
//  Serial.print("distance: "); Serial.println(dist_to_dest);

  //no debouncing, because loop times are long enough
  if(HIGH == digitalRead(BUTTON_1)) // if pressed
  {
    //when this button is pressed, switch to compass mode (i.e. arrow points North, not to destination)
    //bearing = 0;
    if(device_mode == NORMAL_MODE)
      device_mode = COMPASS_MODE;
  }
  else {
    if(device_mode == COMPASS_MODE)
      device_mode = NORMAL_MODE;
  }
  if(HIGH == digitalRead(BUTTON_2)) // if pressed
  { 
    //fake location to ~statues in north quad
    curr_lat = 40.114961/180*PI;
    curr_lon = -88.227322/180*PI;
    fake_fix = true;
  }
  else { fake_fix = false; }
  if(HIGH == digitalRead(BUTTON_3)) // if pressed
  {
    if(!button3) {
      button3 = true;
      // when this button is pressed, update the current location being navigated to
      curr_loc++;
      if(curr_loc == NUM_LOCS)
        curr_loc = 0;
      updateLocText(curr_loc);
      dest_lat = locs[curr_loc][0];
      dest_lon = locs[curr_loc][1];
    }
  }
  else
  {
    if(button3)
      button3 = false;
  }
  // if button 4 held for a long press, set the current location
  if(HIGH == digitalRead(BUTTON_4))
  {
    if(millis() - loc_reset_timer > BUTTON_PRESS_LONG) {
      locs[curr_loc][0] = random(180) - 90;
      locs[curr_loc][1] = random(360) - 180;
      eeprom_write_block((const void*)&locs, (void*)0, 6*sizeof(float));  // update locs in EEPROM
      Serial.println("LONG PRESS 4");
      Serial.print("Loc "); Serial.print(curr_loc); Serial.print(" set to: "); Serial.print(locs[curr_loc][0]); Serial.print(", "); Serial.println(locs[curr_loc][1]);
      loc_reset_timer = millis();
    }
  }

  // if millis() or timer wraps around, we'll just reset it
  if (display_timer > millis())  display_timer = millis();

  // approximately every <TODO, ~400ms is okay...> seconds or so, print out the current stats
  if (millis() - display_timer > 150) { 
    display_timer = millis(); // reset the timer  \

    // print current device mode
    tft.setCursor(MODE_START_X, MODE_START_Y);
    switch(device_mode) {
      case NORMAL_MODE:   tft.println("Normal mode "); break;
      case COMPASS_MODE:  tft.println("Compass mode"); break;
      case DEBUG_MODE:    tft.println("Debug mode  "); break;
    }
    
    //get orientation, calls lsm.read()
    orientation = my_lib::calc_heading(lsm, mag_min, mag_max);  
    
    // Calculate the angle of the vector y,x in radians
    //orientation = atan2(y,x); // NOPE, not anymore
    orientation += getMagDeclination();
    
    // Normalize to 0-360
    if (orientation < 0)
    {
      orientation += 2*PI;
    }
    smoothed_orientation = (orientation * (1 - filterVal)) + (smoothed_orientation  *  filterVal); // smooth orientation
    orientation = smoothed_orientation;                
    //Serial.print("Device Orientation: ");
    //Serial.println(orientation*180/PI);
    
//    tft.drawLine(center_x, center_y, center_x + RAD*sin(display_bearing), center_y - RAD*cos(display_bearing), bg_color); // clear old line

    tft.drawLine(center_x + ARROW_RAD*sin(display_bearing), center_y - ARROW_RAD*cos(display_bearing),
      center_x + ARROW_RAD2*sin(display_bearing + ARROW_RAD2_ANGLE), center_y - ARROW_RAD2*cos(display_bearing + ARROW_RAD2_ANGLE), BG_COLOR_DEFAULT);
    tft.drawLine(center_x + ARROW_RAD*sin(display_bearing), center_y - ARROW_RAD*cos(display_bearing),
      center_x + ARROW_RAD2*sin(display_bearing - ARROW_RAD2_ANGLE), center_y - ARROW_RAD2*cos(display_bearing - ARROW_RAD2_ANGLE), BG_COLOR_DEFAULT);
    if (device_mode == COMPASS_MODE)
    {
      //point North
      display_bearing = -orientation; 
    }
    else
    {
      display_bearing = bearing - orientation; // bearing w/ respect to device orientation
    }
    //Serial.print("Relative Bearing: ");
    //Serial.print(abs(display_bearing*180/PI));
    //Serial.print(" : ");
    //Serial.println(GUIDELINE_ANGLE);

    if(abs(display_bearing*180/PI) < GUIDELINE_ANGLE) {
      if (line_color != LINE_COLOR_SUCCESS) {
        line_color = LINE_COLOR_SUCCESS;
      }
    }
    else {
      if (line_color != LINE_COLOR_DEFAULT) {
        line_color = LINE_COLOR_DEFAULT;
      }
    }
    
    //might have to reverse/transform direction depending on relative orientations of display to magnetometer
    //Y IS NEGATED BECAUSE HIGHER Y VALUES ARE LOWER ON THE SCREEN!!! NOT CARTESIAN
//    tft.drawLine(center_x, center_y, center_x + RAD*sin(display_bearing), center_y - RAD*cos(display_bearing), line_color); //TODO::CHECK sin and cos
    
    tft.drawLine(center_x + ARROW_RAD*sin(display_bearing), center_y - ARROW_RAD*cos(display_bearing),
      center_x + ARROW_RAD2*sin(display_bearing + ARROW_RAD2_ANGLE), center_y - ARROW_RAD2*cos(display_bearing + ARROW_RAD2_ANGLE), line_color);
    tft.drawLine(center_x + ARROW_RAD*sin(display_bearing), center_y - ARROW_RAD*cos(display_bearing),
      center_x + ARROW_RAD2*sin(display_bearing - ARROW_RAD2_ANGLE), center_y - ARROW_RAD2*cos(display_bearing - ARROW_RAD2_ANGLE), line_color);
  }
}
