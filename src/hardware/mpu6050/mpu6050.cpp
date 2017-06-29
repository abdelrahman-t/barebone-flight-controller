//by: Kris Winer
//date: May 10, 2014
//license: Beerware - Use this code however you'd like. If you
//find it useful you can buy me a beer some time.

#include "../lib/i2c_t3/i2c_t3.h"
#include "mpu6050.h"
#include "../../AHRS/madgwickQuaternionFilter.h"

uint8_t imuReady;
int Gscale = GFS_500DPS;
int Ascale = AFS_8G;
float aRes = 8.0 / 32768.0, gRes = 500.0 / 32768.0;

int intPin = 12;
int blinkOn = 0;

int16_t accelCount[3];
float ax, ay, az;
int16_t gyroCount[3];
float gx, gy, gz;
float gyroBias[3] = { 0, 0, 0 }, accelBias[3] = { 0, 0, 0 };
int16_t tempCount;
float temperature;
float SelfTest[6];

float GyroMeasError = PI * (40.0f / 180.0f);
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;
float GyroMeasDrift = PI * (2.0f / 180.0f);
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
float orientationEuler[3];
float deltat = 0.0f;
uint32_t lastUpdate = 0, firstUpdate = 0;
uint32_t Now = 0;
float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };

unsigned long count;

void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];
  readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];
  readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
  destination[0] = (int16_t)((rawData[0] << 8) | rawData[1]);
  destination[1] = (int16_t)((rawData[2] << 8) | rawData[3]);
  destination[2] = (int16_t)((rawData[4] << 8) | rawData[5]);
}

int16_t readTempData()
{
  uint8_t rawData[2];
  readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);
  return ((int16_t)rawData[0]) << 8 | rawData[1];
}

void initMPU6050()
{
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, CONFIG, 0x03);

  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);

  uint8_t c = readByte(MPU6050_ADDRESS, GYRO_CONFIG);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | Gscale << 3);

  c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | Ascale << 3);

  writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);
}

void calibrateMPU6050(float * dest1, float * dest2)
{
  uint8_t data[12];
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80);
  delay(100);

  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
  delay(200);

  writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);
  writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
  writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00);
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);
  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);
  delay(15);

  writeByte(MPU6050_ADDRESS, CONFIG, 0x01);
  writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);
  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00);

  uint16_t  gyrosensitivity = 131;
  uint16_t  accelsensitivity = 16384;

  writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);
  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);
  delay(80);

  writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);
  readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]);
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12;

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
    readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]);
    accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
    accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    accel_bias[0] += (int32_t)accel_temp[0];
    accel_bias[1] += (int32_t)accel_temp[1];
    accel_bias[2] += (int32_t)accel_temp[2];
    gyro_bias[0] += (int32_t)gyro_temp[0];
    gyro_bias[1] += (int32_t)gyro_temp[1];
    gyro_bias[2] += (int32_t)gyro_temp[2];

  }
  accel_bias[0] /= (int32_t)packet_count;
  accel_bias[1] /= (int32_t)packet_count;
  accel_bias[2] /= (int32_t)packet_count;
  gyro_bias[0] /= (int32_t)packet_count;
  gyro_bias[1] /= (int32_t)packet_count;
  gyro_bias[2] /= (int32_t)packet_count;

  if (accel_bias[2] > 0L) { accel_bias[2] -= (int32_t)accelsensitivity; }
  else { accel_bias[2] += (int32_t)accelsensitivity; }

  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
  data[1] = (-gyro_bias[0] / 4) & 0xFF;
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

  writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);// might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
  writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
  writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

  dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
  dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
  dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

  int32_t accel_bias_reg[3] = { 0, 0, 0 };
  readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
  readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
  accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

  uint32_t mask = 1uL;
  uint8_t mask_bit[3] = { 0, 0, 0 };

  for (ii = 0; ii < 3; ii++) {
    if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;
  }

  accel_bias_reg[0] -= (accel_bias[0] / 8);
  accel_bias_reg[1] -= (accel_bias[1] / 8);
  accel_bias_reg[2] -= (accel_bias[2] / 8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  data[1] = data[1] | mask_bit[0];
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  data[3] = data[3] | mask_bit[1];
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  data[5] = data[5] | mask_bit[2];

  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]); // might not be supported in MPU6050
  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

  dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
  dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
  dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}


void MPU6050SelfTest(float * destination)
{
  uint8_t rawData[4];
  uint8_t selfTest[6];
  float factoryTrim[6];

  writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0);
  writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0xE0);
  delay(250);
  rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X);
  rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y);
  rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z);
  rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A);

  selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4;
  selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2;
  selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03);

  selfTest[3] = rawData[0] & 0x1F;
  selfTest[4] = rawData[1] & 0x1F;
  selfTest[5] = rawData[2] & 0x1F;

  factoryTrim[0] = (4096.0*0.34)*(pow((0.92 / 0.34), (((float)selfTest[0] - 1.0) / 30.0)));
  factoryTrim[1] = (4096.0*0.34)*(pow((0.92 / 0.34), (((float)selfTest[1] - 1.0) / 30.0)));
  factoryTrim[2] = (4096.0*0.34)*(pow((0.92 / 0.34), (((float)selfTest[2] - 1.0) / 30.0)));
  factoryTrim[3] = (25.0*131.0)*(pow(1.046, ((float)selfTest[3] - 1.0)));
  factoryTrim[4] = (-25.0*131.0)*(pow(1.046, ((float)selfTest[4] - 1.0)));
  factoryTrim[5] = (25.0*131.0)*(pow(1.046, ((float)selfTest[5] - 1.0)));

  for (int i = 0; i < 6; i++) {
    destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i];
  }

}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t)1);
  data = Wire.read();
  return data;
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(address, count);
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }
}
void startImu(){
  uint8_t c = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);

  if (c == 0x68)
  {
    Serial.println("MPU6050 is online...");

    MPU6050SelfTest(SelfTest);

    if (SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {

      calibrateMPU6050(gyroBias, accelBias);

      initMPU6050(); Serial.println("MPU6050 initialized for active data mode....");
    }
    else
    {
      Serial.print("Could not connect to MPU6050: 0x");
      Serial.println(c, HEX);
      while (1);
    }
  }
}

void getRotationEuler()
{
  if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {
    readAccelData(accelCount);

    ax = (float)accelCount[0] * aRes;
    ay = (float)accelCount[1] * aRes;
    az = (float)accelCount[2] * aRes;

    readGyroData(gyroCount);

    gx = (float)gyroCount[0] * gRes;
    gy = (float)gyroCount[1] * gRes;
    gz = (float)gyroCount[2] * gRes;

    tempCount = readTempData();
    temperature = ((float)tempCount) / 340. + 36.53;
  }

  Now = micros();
  deltat = ((Now - lastUpdate) / 1000000.0f);
  lastUpdate = Now;

  MadgwickQuaternionUpdate(ax, ay, az, gx*PI / 180.0f, gy*PI / 180.0f, gz*PI / 180.0f);

  orientationEuler[YAW] = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  orientationEuler[PITCH] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  orientationEuler[ROLL] = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

  orientationEuler[YAW] *= 180.0f / PI;
  orientationEuler[PITCH] *= 180.0f / PI;
  orientationEuler[ROLL]  *= 180.0f / PI;

#ifdef DEBUG
  Serial.print("\n Yaw: ");
  Serial.print(orientationEuler[YAW], 2);
  Serial.print("\t Pitch: ");
  Serial.print(orientationEuler[PITCH], 2);
  Serial.print("\t Roll: ");
  Serial.println(orientationEuler[ROLL], 2);
#endif


  count = millis();
}
