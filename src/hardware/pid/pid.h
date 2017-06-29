#include "../../common/common.h"

extern float orientationEuler[3];
extern float gains[3][3];
extern float setPoints[3], pidOut[3], pidMaxOut[3];
extern float currentError[3], accumulatedError[3], previousError[3];

extern uint16_t motorsPidCorrections[NUMBER_OF_MOTORS];
extern uint16_t throttle;
extern uint_fast32_t lastTime;
extern uint16_t sampleTime;

uint8_t calculatePid();
