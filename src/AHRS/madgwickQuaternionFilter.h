// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
extern float q[4];
extern float beta;
extern float zeta;
extern float deltat;

float invSqrt(float x);
void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz);
