
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include <math.h>

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)
#define YAW_SETPOINT_THRESH 30.0f 	// User can change set yaw orientation after
										// rotating + or - 30 degrees

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

float yawError;
void controllerPid(control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Rate-controlled YAW is moving YAW angle setpoint
	  /** WARNING! EXPERIMENTALLY MODIFIED FOR ONLY CORRECTING YAW ON 30 DEG. INTERVALS: **/
	// if (yawError > YAW_SETPOINT_THRESH)
	// 	setpoint->attitude.yaw += yawError;
	// else if (yawError < -(YAW_SETPOINT_THRESH))
	// 	setpoint->attitude.yaw -= yawError;

    if (setpoint->mode.yaw == modeVelocity) {
      attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;
       // attitudeDesired.yaw = setpoint->attitude.yaw;
      while (attitudeDesired.yaw > 180.0f)
        attitudeDesired.yaw -= 360.0f;
      while (attitudeDesired.yaw < -180.0f)
        attitudeDesired.yaw += 360.0f;
    } else {
    	attitudeDesired.yaw = setpoint->attitude.yaw;
    }

  }

  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {
      actuatorThrust = setpoint->thrust;
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
      attitudeDesired.roll = setpoint->attitude.roll;
      attitudeDesired.pitch = setpoint->attitude.pitch;
    }

//    float yawError; <-- made global

    /** WARNING! EXPERIMENTALLY MODIFIED FOR RESETTING YAW SETPOINT AT 30 DEG. INTERVALS: **/
    float eulerYawActual = state->attitude.yaw;
    double yawDesDelta = (double)(eulerYawActual - attitudeDesired.yaw);
    double yawSetDelta = (double)(eulerYawActual - setpoint->attitude.yaw);
    if ((fabs(yawDesDelta) > (double)YAW_SETPOINT_THRESH) || (fabs(yawSetDelta) > (double)YAW_SETPOINT_THRESH)) {
        attitudeDesired.yaw = eulerYawActual;
        setpoint->attitude.yaw = eulerYawActual;
        control->yaw = eulerYawActual;
    }
    /**    **/
    
    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw,
								&yawError);

    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {
      rateDesired.roll = setpoint->attitudeRate.roll;
      attitudeControllerResetRollAttitudePID();
    }
    if (setpoint->mode.pitch == modeVelocity) {
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

    attitudeControllerGetActuatorOutput(&control->roll,
                                        &control->pitch,
                                        &control->yaw);

    control->yaw = -control->yaw;
    // ^ inverted orientation for sensor inputs
  }

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt() /* my addition: */ / 2;
    // sensfusion6GetInvThrustCompensationForTilt: Returns the z component of the estimated gravity direction
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

// From log.h:
LOG_GROUP_START(controller)
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

// From param.h:
PARAM_GROUP_START(controller)
// TODO: Investigate the computation chain for tiltComp
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
