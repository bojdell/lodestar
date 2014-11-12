#include <my_lib.h>

//returns tilt compensated, offset corrected heading in radians
float my_lib::calc_heading(Adafruit_LSM303 lsm, int16_t* mag_min, int16_t* mag_max)
{
	float from[3] = {1, 0, 0}; //calculate heading w/ respect to positive X direction
	//read from the lsm
    lsm.read();
	
	//put data into arrays
	/*
	int32_t mag[3] = [(int)lsm.magData.x,   (int)lsm.magData.y,   (int)lsm.magData.z];
	int32_t acc[3] = [(int)lsm.accelData.x, (int)lsm.accelData.y, (int)lsm.accelData.z];
	*/
	//all float implementation
	float mag[3] = {(int)lsm.magData.x,   (int)lsm.magData.y,   (int)lsm.magData.z};
	float acc[3] = {(int)lsm.accelData.x, (int)lsm.accelData.y, (int)lsm.accelData.z};
	
	//offset correction
	mag_offset_correction(mag, mag_min, mag_max);

	float North[3];
	float East[3];

	cross_product(mag, acc, East);
	normalize(East);
	cross_product(acc, East, North);
	normalize(North);
	
	float heading = atan2(dot_product(East, from), dot_product(North, from));
    if (heading < 0) heading += 2*PI;
	
	return heading;
}

void my_lib::mag_offset_correction(float* mag, int16_t* mag_min, int16_t* mag_max)
{
	//offset correction
	mag[0] -= ((int32_t)mag_min[0] + mag_max[0]) / 2;
	mag[1] -= ((int32_t)mag_min[1] + mag_max[1]) / 2;
	mag[2] -= ((int32_t)mag_min[2] + mag_max[2]) / 2;
}
void my_lib::normalize(float* a)
{
  float mag = sqrt(dot_product(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

float my_lib::dot_product(float* a, float* b)
{
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void my_lib::cross_product(float* a, float* b, float* output)
{
  output[0] = (a[1] * b[2]) - (a[2] * b[1]);
  output[1] = (a[2] * b[0]) - (a[0] * b[2]);
  output[2] = (a[0] * b[1]) - (a[1] * b[0]);
}

//////////
/*
template <typename T> float LSM303::heading(vector<T> from)
{
    vector<int32_t> temp_m = {m.x, m.y, m.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

    // compute E and N
    vector<float> E;
    vector<float> N;
    vector_cross(&temp_m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
    if (heading < 0) heading += 360;
    return heading;
}

template <typename Ta, typename Tb, typename To> void LSM303::vector_cross(const vector<Ta> *a,const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float LSM303::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}

void LSM303::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}
*/
