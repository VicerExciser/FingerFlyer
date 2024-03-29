/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * pid.h - implementation of the PID regulator
 */
#ifndef PID_H_
#define PID_H_

#include <stdbool.h>
#include "filter.h"

//#if defined(PLATFORM_CF2)

#define RP_RATE_KP 250.0  	///* tried: 200.0(good), 150.0(DONT USE->infinite thrust) *////* stock: 250.0 *///
#define RP_RATE_KI 500.0 	///* tried:  *////* stock: 500.0 *///
#define RP_RATE_KD 2.5 		///* tried:  *////* stock: 2.5*///

#define PID_ROLL_RATE_KP  RP_RATE_KP
#define PID_ROLL_RATE_KI  RP_RATE_KI
#define PID_ROLL_RATE_KD  RP_RATE_KD
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  RP_RATE_KP
#define PID_PITCH_RATE_KI  RP_RATE_KI
#define PID_PITCH_RATE_KD  RP_RATE_KD
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  120.0 //100.0 	///* tried: 60.0 , 70.0(good), 75.0(possible problematic) *////* stock: 120.0 *///
#define PID_YAW_RATE_KI  16.7 	///* tried:  */
#define PID_YAW_RATE_KD  0.0 	///* tried:  */
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7



/* Experimental values found from https://github.com/bitcraze/crazyflie-firmware/commit/a0628ec2c171c0cac18bdf5306101c697ebae590 */
/*
 * Lower P gain == less overcompensation by stabilizer
 */

#define RP_KP 6.75 //7.0 //8.5 //7.5 //9.5 //11.0  ///* tried: 16.0, 8.0, 8.5(great) 9.0, 10.0(good), 12.0(aight) *////* stock: 6.0 *///
#define RP_KI 3.0 //2.5 //3.75 //3.0 //1.75 //5.5 //5.0 //4.5 ///* tried: 5.5(great), 4.0(good), 3.5(aight), 6.0(test more), 1.0(too slow to react-especially for freefall), 2.5(barely too slow) *////* stock: 3.0 *///
#define RP_KD 0.0 		///* tried: 0.5(bad), 0.1(dont think so..) *////* stock: 0.0 *///

#define PID_ROLL_KP  RP_KP
#define PID_ROLL_KI  RP_KI
#define PID_ROLL_KD  RP_KD
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  RP_KP
#define PID_PITCH_KI  RP_KI
#define PID_PITCH_KD  RP_KD
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  7.0 //7.5	///* tried: 7.0(great) 10.0, 6.5(unstable) *////* stock: 6.0 *///
#define PID_YAW_KI  1.75 	///* tried: 2.0(BAD), 1.5(great) *////* stock: 1.0 *///
#define PID_YAW_KD  0.0 //0.35 	///* tried: 0.1(great) *////* stock: 0.35 *///
#define PID_YAW_INTEGRATION_LIMIT     360.0


#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
#define DEFAULT_PID_OUTPUT_LIMIT      0.0

//#else // (for CFv1, obsolete)
/*
#define PID_ROLL_RATE_KP  70.0
#define PID_ROLL_RATE_KI  0.0
#define PID_ROLL_RATE_KD  0.0
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3

#define PID_PITCH_RATE_KP  70.0
#define PID_PITCH_RATE_KI  0.0
#define PID_PITCH_RATE_KD  0.0
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3

#define PID_YAW_RATE_KP  70.0
#define PID_YAW_RATE_KI  16.7
#define PID_YAW_RATE_KD  0.0
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7

#define PID_ROLL_KP  3.5
#define PID_ROLL_KI  2.0
#define PID_ROLL_KD  0.0
#define PID_ROLL_INTEGRATION_LIMIT    20.0

#define PID_PITCH_KP  3.5
#define PID_PITCH_KI  2.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

#define PID_YAW_KP  10.0
#define PID_YAW_KI  1.0
#define PID_YAW_KD  0.35
#define PID_YAW_INTEGRATION_LIMIT     360.0


#define DEFAULT_PID_INTEGRATION_LIMIT 10000.0
#define DEFAULT_PID_OUTPUT_LIMIT      0.0
*/
//#endif

typedef struct
{
  float desired;      //< set point
  float error;        //< error
  float prevError;    //< previous error
  float integ;        //< integral
  float deriv;        //< derivative
  float kp;           //< proportional gain
  float ki;           //< integral gain
  float kd;           //< derivative gain
  float outP;         //< proportional output (debugging)
  float outI;         //< integral output (debugging)
  float outD;         //< derivative output (debugging)
  float iLimit;       //< integral limit, absolute value. '0' means no limit.
  float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
  float dt;           //< delta-time dt
  lpf2pData dFilter;  //< filter for D term
  bool enableDFilter; //< filter for D term enable flag
} PidObject;

/**
 * PID object initialization.
 *
 * @param[out] pid   A pointer to the pid object to initialize.
 * @param[in] desired  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 * @param[in] cutoffFreq   Frequency to set the low pass filter cutoff at
 * @param[in] enableDFilter Enable setting for the D lowpass filter
 */
 void pidInit(PidObject* pid, const float desired, const float kp,
              const float ki, const float kd, const float dt,
              const float samplingRate, const float cutoffFreq,
              bool enableDFilter);

/**
 * Set the integral limit for this PID in deg.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidSetIntegralLimit(PidObject* pid, const float limit);

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] limit Pid integral swing limit.
 */
void pidReset(PidObject* pid);

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 */
float pidUpdate(PidObject* pid, const float measured, const bool updateError);

/**
 * Set a new set point for the PID to track.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] angle The new set point
 */
void pidSetDesired(PidObject* pid, const float desired);

/**
 * Set a new set point for the PID to track.
 * @return The set point
 */
float pidGetDesired(PidObject* pid);

/**
 * Find out if PID is active
 * @return TRUE if active, FALSE otherwise
 */
bool pidIsActive(PidObject* pid);

/**
 * Set a new error. Use if a special error calculation is needed.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] error The new error
 */
void pidSetError(PidObject* pid, const float error);

/**
 * Set a new proportional gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kp    The new proportional gain
 */
void pidSetKp(PidObject* pid, const float kp);

/**
 * Set a new integral gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] ki    The new integral gain
 */
void pidSetKi(PidObject* pid, const float ki);

/**
 * Set a new derivative gain for the PID.
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] kd    The derivative gain
 */
void pidSetKd(PidObject* pid, const float kd);

/**
 * Set a new dt gain for the PID. Defaults to IMU_UPDATE_DT upon construction
 *
 * @param[in] pid   A pointer to the pid object.
 * @param[in] dt    Delta time
 */
void pidSetDt(PidObject* pid, const float dt);
#endif /* PID_H_ */


