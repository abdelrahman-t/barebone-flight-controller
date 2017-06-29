#include "fc.h"
#include "common/common.h"
#include "../lib/serialprint/serialprint.h"
#include "../lib/i2c_t3/i2c_t3.h"
#include "hardware/mpu6050/mpu6050.h"
#include "hardware/pid/pid.h"

#define DEBUG
/* ROLL  {P, I, D}
   PITCH {P, I, D}
   YAW   {P, I, D} */
float gains[3][3] = { {1.4, 0.05, 15.0},
                      {1.4, 0.05, 15.0},
                      {0.0, 0.0 , 0.0 } };

float pidMaxOut[3] = {400.0, 400.0, 400.0};

uint8_t isMotorArmed(Motor *selectedMotor){
  return (selectedMotor->isArmed);
}

uint8_t isMotorCalibrated(Motor *selectedMotor){
  return (selectedMotor->isCalibrated);
}

uint8_t calibrateMotor(Motor *selectedMotor)
{
  analogWrite(selectedMotor->motorPin, MAX_ANALOG_WRITE);
  delay(2500);
  analogWrite(selectedMotor->motorPin, MIN_ANALOG_WRITE);
  delay(1000);

  disarmMotor(selectedMotor);
  selectedMotor->isCalibrated = 1;
  #ifdef DEBUG
    Serialprint("[!] Motor [%hu] is CALIBRATED\n", selectedMotor->motorNumber);
  // TODO : return current PinMode as this return is useless
  #endif
  return 0;
}

uint8_t calibrateMotors(){
  for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)
    calibrateMotor(&motors[i]);

  return 0;
}

uint8_t disarmMotor(Motor *selectedMotor)
{
  analogWrite(selectedMotor->motorPin, MIN_ANALOG_WRITE);
  pinMode(selectedMotor->motorPin, INPUT);
  #ifdef DEBUG
  Serialprint("[!] Motor %hu is DISARMED\n", selectedMotor->motorNumber);
  #endif
  // TODO : return current PinMode as this is unsafe
  return !(selectedMotor->isArmed = 0);
}

uint8_t armMotor(Motor *selectedMotor)
{
  motorSpeed(selectedMotor, MIN_PULSE_WIDTH);
  #ifdef DEBUG
  Serialprint("[!!!] Motor %hu is ARMED\n", selectedMotor->motorNumber);
  #endif
  return !(selectedMotor->isArmed = 1);
}

uint8_t armMotors(){
  for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)
    armMotor(&motors[i]);

  return 0;
}

uint16_t motorSpeed(Motor *selectedMotor, uint16_t absoluteSpeed)
{
  if (!isMotorArmed(selectedMotor))
    #ifdef DEBUG
    return Serialprint("[X] Unable to start Motor [%hu] UNARMED\n",
    selectedMotor->motorNumber), 1;
    #endif

  if (!isMotorCalibrated(selectedMotor))
    #ifdef DEBUG
    return Serialprint("[X] Unable to start Motor [%hu] UNCALIBRATED\n",
    selectedMotor->motorNumber), 1;
    #endif

  selectedMotor->motorSpeed = map_PWM(absoluteSpeed);
  analogWrite(selectedMotor->motorPin, selectedMotor->motorSpeed);

#ifdef DEBUG
  uint8_t throttlePercentage = (float)(selectedMotor->motorSpeed - MIN_ANALOG_WRITE) * 100.0
                             / (float)(MAX_ANALOG_WRITE-MIN_ANALOG_WRITE);

  Serialprint("[!] Motor [%hu] Speed is updated to (%hu%%)\n",
  selectedMotor->motorNumber, throttlePercentage);
#endif
  return absoluteSpeed;
}

uint8_t motorSpeeds(uint16_t absoluteSpeed[NUMBER_OF_MOTORS]){
  for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++)
    motorSpeed(&motors[i], absoluteSpeed[i]);

  return 0;
}

uint16_t step(Motor *selectedMotor ,int16_t step){
  return motorSpeed(selectedMotor, getMotorSpeed(selectedMotor)+step);
}

uint16_t getMotorSpeed(Motor *selectedMotor){
  return selectedMotor->motorSpeed;
}

uint16_t map_PWM(int16_t output){
  if (output < MIN_SPINING_SPEED && (output = MIN_SPINING_SPEED));
  else if (output > MAX_PULSE_WIDTH && (output = MAX_PULSE_WIDTH));

  return
  ((output - MIN_PULSE_WIDTH) * (MAX_ANALOG_WRITE - MIN_ANALOG_WRITE) /
  (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) + MIN_ANALOG_WRITE;
}

uint8_t startMotors(){
  calibrateMotors(), armMotors();

  uint16_t initialSpeeds[NUMBER_OF_MOTORS] = {MIN_SPINING_SPEED};
  motorSpeeds(initialSpeeds);
  return Serialprint("[!] Motors Started\n"), 0;
}

uint8_t waitForInput(){
  while (1){
    if (Serial.available() > 0)
      break;
  }
  return Serial.read(), 0;
}

uint8_t initializePWM(){
    for (uint8_t i = 0; i < NUMBER_OF_MOTORS; i++){
      motors[i] = (Motor){.motorNumber = i, .motorPin = motorPins[i],
                          .isCalibrated = 0, .isArmed = 0};

      analogWriteFrequency(motorPins[i], PWM_FREQUENCY);
    }
    analogWriteResolution(PWM_RESOLUTION);
    return Serialprint("[!] PWM Initialized\n"), 0;
}

void setup()
{
  Serial.begin(BAUD_RATE);
  waitForInput();

#ifdef DEBUG
  Serialprint("[!] DEBUG MODE ENABLED\n");
#endif

  initializePWM();
  startMotors();
  delay(2000);

  Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  startImu();
  loopStart = millis();
}

void loop()
{
  if (Serial.available() > 2){
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
  }
  Serial.print("what");
  getRotationEuler();
  if(!imuReady){
   if (millis()-loopStart < imuInitializationTime)
     return ;

   imuReady = 1;
   #ifdef DEBUG
     Serialprint("[!] IMU is ready\n");
   #endif
  }

  calculatePid();
  motorSpeeds(motorsPidCorrections);
}
