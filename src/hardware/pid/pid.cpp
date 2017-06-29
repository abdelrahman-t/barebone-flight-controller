#include "pid.h"
#include <Arduino.h>

float setPoints[3] = {0.0}, pidOut[3];
float currentError[3], accumulatedError[3] = {0.0}, previousError[3] = {0.0};
uint16_t sampleTime = 490;
uint_fast32_t lastTime;

uint8_t calculatePid()
{
  uint_fast32_t now = micros();
  uint_fast32_t timeDifference = now - lastTime;
  if (timeDifference >= sampleTime)
  {
    for (uint8_t axis = ROLL; axis < 3; axis++)
    {
      currentError[axis] = orientationEuler[axis] - setPoints[axis];
      accumulatedError[axis] += gains[axis][INTEGRAL] * currentError[axis];

      if(accumulatedError[axis] > pidMaxOut[axis])
        accumulatedError[axis] = pidMaxOut[axis];

      else if(accumulatedError[axis] < -1*pidMaxOut[axis])
        accumulatedError[axis] = -1*pidMaxOut[axis];

      pidOut[axis] = gains[axis][PROPORTIONAL] * currentError[axis] +
                     accumulatedError[axis] +
                     gains[axis][DERIVATIVE] * (currentError[axis]-previousError[axis]);

      if(pidOut[axis] > pidMaxOut[axis])
        pidOut[axis] = pidMaxOut[axis];

      else if(pidOut[axis] < -1*pidMaxOut[axis])
        pidOut[axis] = -1*pidMaxOut[axis];

      previousError[axis] = currentError[axis];
    }

    //(front-right - CCW)
    motorsPidCorrections[0] = throttle - pidOut[PITCH] + pidOut[ROLL] - pidOut[YAW];
    //(rear-right - CW)
    motorsPidCorrections[1] = throttle + pidOut[PITCH] + pidOut[ROLL] + pidOut[YAW];
    //(rear-left - CCW)
    motorsPidCorrections[2] = throttle + pidOut[PITCH] - pidOut[ROLL] - pidOut[YAW];
    //(front-left - CW)
    motorsPidCorrections[3] = throttle - pidOut[PITCH] - pidOut[ROLL] + pidOut[YAW];
  }

  lastTime = now;
  return 0;
}
