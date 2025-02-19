#ifndef MOTOR_H
#define MOTOR_H

/*
  Motor.h

  A simple "Motor" class that allows you to set the desired
  position and time units, then call a method (like trapezoidMove)
  using those units. Internally, it will convert to "encoder_counts"
  for position and "timesteps" for time (or whichever internal units
  you need).

  Requirements:
    - Include ArduinoEmulator.h and AutoGeneratedUnitConversions.h
      so that this code compiles and runs on your Mac with the 
      same usage as on an Arduino.
*/

#include "ArduinoEmulator.h"
#include "AutoGeneratedUnitConversions.h"

class Motor {
public:
    Motor();

    // Set the position and time units used by the Motor's commands
    void setPositionUnit(PositionUnit unit);
    void setTimeUnit(TimeUnit unit);

    // Example motion function that uses the chosen units
    // distance: the position value in the chosen position unit
    // duration: the time value in the chosen time unit
    void trapezoidMove(float distance, float duration);

private:
    PositionUnit m_positionUnit;
    TimeUnit     m_timeUnit;

    // You could also store velocityUnit, accelerationUnit, etc. 
    // For demonstration, we'll keep it simple.
};

#endif // MOTOR_H