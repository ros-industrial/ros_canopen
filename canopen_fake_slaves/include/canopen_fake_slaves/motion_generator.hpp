#ifndef __MOTIONGENERATOR_H__
#define __MOTIONGENERATOR_H__

// Copyright (c) 2016 AerDronix, https://aerdronix.wordpress.com/
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <chrono>
/**
 * Generates the analytical solution for the trapezoidal motion.
 *
 * <p>
 * Usage:
 * // Includes
 * #include "MotionGenerator.h"
 *
 * Initialization
 *
 * @param int aVelMax maximum velocity (units/s)
 * @param int aAccMax maximum acceleration (units/s^2)
 * @param int aInitPos initial position (units)
 *
 // Define the MotionGenerator object
 MotionGenerator *trapezoidalProfile = new MotionGenerator(100, 400, 0);
 // Retrieve calculated position
 double positionRef = 1000;
 double position = trapezoidalProfile->update(positionRef)
 // Retrieve current velocity
 double velocity = trapezoidalProfile->getVelocity();
 // Retrieve current acceleration
 double acceleration = trapezoidalProfile->getAcceleration();
 // Check if profile is finished
 if (trapezoidalProfile->getFinished()) {};
 // Reset internal state
 trapezoidalProfile->reset();
 *
 * @author      AerDronix <aerdronix@gmail.com>
 * @web		https://aerdronix.wordpress.com/
 * @version     1.0
 * @since       2016-12-22
 */

class MotionGenerator
{
public:
  /**
   * Constructor
   *
   * @param int aVelocityMax maximum velocity
   * @param int aAccelerationMax maximum acceleration
   */
  MotionGenerator(double aMaxVel, double aMaxAcc, double aInitPos);

  void init();

  /**
   * Updates the state, generating new setpoints
   *
   * @param aSetpoint The current setpoint.
   */
  double update(double aPosRef);
  double getVelocity();
  double getAcceleration();

  bool getFinished();
  void setMaxVelocity(double aMaxVel);
  void setMaxAcceleration(double aMaxAcc);
  void setInitPosition(double aInitPos);
  void reset();

private:
  /**
   * Increments the state number.
   *
   * @see
    currentState
   */
  void calculateTrapezoidalProfile(double);
  short int sign(double aVal);

  double maxVel;
  double maxAcc;
  double initPos;
  double pos;
  double vel;
  double acc;
  double oldPos;
  double oldPosRef;
  double oldVel;

  double dBrk;
  double dAcc;
  double dVel;
  double dDec;
  double dTot;

  double tBrk;
  double tAcc;
  double tVel;
  double tDec;

  double velSt;

  std::chrono::time_point<std::chrono::high_resolution_clock> oldTime;
  std::chrono::time_point<std::chrono::high_resolution_clock> lastTime;
  std::chrono::duration<double> deltaTime;

  short int signM;  // 1 = positive change, -1 = negative change
  bool shape;       // true = trapezoidal, false = triangular

  bool isFinished;
};
#endif
