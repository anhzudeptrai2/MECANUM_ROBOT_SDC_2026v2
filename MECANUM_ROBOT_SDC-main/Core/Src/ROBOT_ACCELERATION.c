#include "ROBOT_ACCELERATION.h"
#include <math.h>

void RobotAcceleration_Init(RobotAcceleration *robot, float initialTarget, float duration)
{
  float safe_duration = (duration > 0.001f) ? duration : 0.001f;

  robot->currentVelocity = initialTarget;
  robot->targetVelocity = initialTarget;
  robot->previousTarget = initialTarget;
  robot->duration = safe_duration;
  robot->tau = safe_duration / 5.0f;
  robot->timeElapsed = safe_duration;
}

void RobotAcceleration_SetTarget(RobotAcceleration *robot, float newTargetVelocity, float newDuration)
{
  if (fabsf(newTargetVelocity - robot->targetVelocity) < 0.5f)
  {
    return;
  }

  robot->previousTarget = robot->currentVelocity;
  robot->targetVelocity = newTargetVelocity;

  {
    float safe_duration = (newDuration > 0.001f) ? newDuration : 0.001f;
    robot->duration = safe_duration;
    robot->tau = safe_duration / 5.0f;
  }

  robot->timeElapsed = 0.0f;
}

float RobotAcceleration_Update(RobotAcceleration *robot, float dt)
{
  robot->timeElapsed += dt;

  if (robot->timeElapsed >= robot->duration)
  {
    robot->timeElapsed = robot->duration;
    robot->currentVelocity = robot->targetVelocity;
    robot->previousTarget = robot->targetVelocity;
    return robot->currentVelocity;
  }

  if (robot->tau <= 0.00001f)
  {
    robot->currentVelocity = robot->targetVelocity;
    return robot->currentVelocity;
  }

  {
    float velocityRange = robot->targetVelocity - robot->previousTarget;
    robot->currentVelocity = robot->previousTarget + velocityRange * (1.0f - expf(-robot->timeElapsed / robot->tau));
  }

  return robot->currentVelocity;
}

