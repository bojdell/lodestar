#include <EEPROM.h>
#include <avr/eeprom.h>

int a = 0;
int value;
float value2 = 0.0;

float locs[3][2] = {
  { 40.102003/180*PI, -88.227170/180*PI },      // UIUC South Quad:  40.102003, -88.227170
  { 40.7056308/180*PI, -73.9780035/180*PI },    // NYC:              40.7056308, -73.9780035
  { 37.7682682/180*PI, -122.4311711/180*PI }    // SF:               37.7682682, -122.4311711
};
float locs2[3][2];

void setup()
{
  Serial.begin(9600);
//  Serial.println((double)(90 * DEG_TO_RAD));
//  eeprom_write_block((const void*)&locs, (void*)0, 6*sizeof(float));
//  delay(500);
  eeprom_read_block((void*)&locs2, (void*)0, 6*sizeof(float));
  
  Serial.println(locs2[0][0]);
  Serial.println(locs2[0][1]);
  Serial.println(locs2[1][0]);
  Serial.println(locs2[1][1]);
  Serial.println(locs2[2][0]);
  Serial.println(locs2[2][1]);
  Serial.println("==================");
  delay(500);
//  for(int i = 0; i < 3; i++) {
//    EEPROM.write(
//  }
}

void loop()
{
  value = EEPROM.read(a);

  Serial.print(a);
  Serial.print("\t");
  Serial.print(value);
  Serial.println();

  a = a + 1;

  if (a == 512)
    a = 0;

  delay(500);
}
