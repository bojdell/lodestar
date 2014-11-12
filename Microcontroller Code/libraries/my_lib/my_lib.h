#ifndef __MY_LIB_H__
#define __MY_LIB_H__
	
#include "Adafruit_LSM303.h"
	
class my_lib
{
  public:
	//static int16_t mag_min[3]; // = {-32767, -32767, -32767}; // FILL THESE IN
	//static int16_t mag_max[3]; // = {+32767, +32767, +32767}; // FILL THESE IN
	
	
    static float calc_heading(Adafruit_LSM303 lsm, int16_t* mag_min, int16_t* mag_max);
	
  private:
	static void mag_offset_correction(float* mag, int16_t* mag_min, int16_t* mag_max);
  
	static void normalize(float* a);

	static float dot_product(float* a, float* b);

	static void cross_product(float* a, float* b, float* output);

    /*
	template <typename T> struct vector
    {
      T x, y, z;
    };
	
	vector<int16_t> a; // accelerometer readings
    vector<int16_t> m; // magnetometer readings
    vector<int16_t> m_max; // maximum magnetometer values, used for calibration
    vector<int16_t> m_min; // minimum magnetometer values, used for calibration
	
	
	template <typename T> float heading(vector<T> from);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a,const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);
	*/
};	
#endif
