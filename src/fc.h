#include <inttypes.h>
#include "common/common.h"

#define BAUD_RATE 9600

#define PWM_FREQUENCY 400
#define PWM_RESOLUTION 16
#define MIN_PULSE_WIDTH 1000
#define MAX_PULSE_WIDTH 2000
#define MAX_ANALOG_WRITE 52429
#define MIN_ANALOG_WRITE 26213
#define MIN_SPINING_SPEED 1100

uint_fast32_t loopStart;

const uint8_t motorPins[NUMBER_OF_MOTORS] = {3, 4, 5, 6};
uint16_t motorsPidCorrections[NUMBER_OF_MOTORS];
extern float gains[3][3];
extern float pidMaxOut[3];
uint16_t throttle;

typedef struct
{
  uint8_t motorNumber;
  uint8_t motorPin;
  uint8_t isCalibrated;
  uint8_t isArmed;
  uint16_t motorSpeed;

} Motor;

Motor motors[NUMBER_OF_MOTORS];

uint8_t calibrateMotor(Motor *selectedMotor);

uint8_t armMotor(Motor *selectedMotor);
uint8_t armMotors();

uint8_t disarmMotor(Motor *selectedMotor);

uint16_t motorSpeed(Motor *selectedMotor, uint16_t absoluteSpeed);
uint8_t motorSpeeds(uint16_t absoluteSpeed[NUMBER_OF_MOTORS]);

uint16_t step(Motor *selectedMotor ,int16_t step, unsigned char scaler);

uint16_t getMotorSpeed(Motor *selectedMotor);
uint8_t isMotorArmed(Motor *selectedMotor);
uint8_t isMotorCalibrated(Motor *selectedMotor);

uint16_t map_PWM(int16_t output);
uint8_t calibrateMotors();

uint8_t startMotors();

uint8_t initializePWM();
uint8_t waitForInput();
