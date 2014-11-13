#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_LSM303_U.h>
#include <LSM303.h>

/* Assign a unique ID to this sensor at the same time */
LSM303 accel = Adafruit_LSM303_Accel_Unified(54321);

//template <typename T> struct vector
//{
//  T x, y, z;
//};
//
//vector<int16_t> a; // accelerometer readings
//vector<int16_t> m; // magnetometer readings
//vector<int16_t> m_max; // maximum magnetometer values, used for calibration
//vector<int16_t> m_min; // minimum magnetometer values, used for calibration

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) 
{
  Serial.begin(9600);
  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
//  m_min = (vector<int16_t>){-32767, -32767, -32767};
//  m_max = (vector<int16_t>){+32767, +32767, +32767};
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

void loop(void) 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);
  
  float h = heading();
   
  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
  Serial.print("HEADING: "); Serial.println(h); Serial.println(" ");
  delay(500);
}

///*
//Returns the angular difference in the horizontal plane between a
//default vector and north, in degrees.
//The default vector here is chosen to point along the surface of the
//PCB, in the direction of the top of the text on the silkscreen.
//This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
//the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
//*/
//float getHeading(void)
//{
//    return heading((vector<int>){1, 0, 0});
//}
//
///*
//Returns the angular difference in the horizontal plane between the
//"from" vector and north, in degrees.
//Description of heading algorithm:
//Shift and scale the magnetic reading based on calibration data to find
//the North vector. Use the acceleration readings to determine the Up
//vector (gravity is measured as an upward acceleration). The cross
//product of North and Up vectors is East. The vectors East and North
//form a basis for the horizontal plane. The From vector is projected
//into the horizontal plane and the angle between the projected vector
//and horizontal north is returned.
//*/
//template <typename T> float heading(vector<T> from)
//{
//    vector<int32_t> temp_m = {m.x, m.y, m.z};
//
//    // subtract offset (average of min and max) from magnetometer readings
//    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
//    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
//    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;
//
//    // compute E and N
//    vector<float> E;
//    vector<float> N;
//    vector_cross(&temp_m, &a, &E);
//    vector_normalize(&E);
//    vector_cross(&a, &E, &N);
//    vector_normalize(&N);
//
//    // compute heading
//    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
//    if (heading < 0) heading += 360;
//    return heading;
//}
//
//template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a,const vector<Tb> *b, vector<To> *out)
//{
//  out->x = (a->y * b->z) - (a->z * b->y);
//  out->y = (a->z * b->x) - (a->x * b->z);
//  out->z = (a->x * b->y) - (a->y * b->x);
//}
//
//template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
//{
//  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
//}
//
//void vector_normalize(vector<float> *a)
//{
//  float mag = sqrt(vector_dot(a, a));
//  a->x /= mag;
//  a->y /= mag;
//  a->z /= mag;
//}
//

