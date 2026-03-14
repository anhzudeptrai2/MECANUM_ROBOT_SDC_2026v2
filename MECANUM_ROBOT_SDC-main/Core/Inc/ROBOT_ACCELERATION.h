#ifndef ROBOT_ACCELERATION_H
#define ROBOT_ACCELERATION_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float currentVelocity;
  float targetVelocity;
  float previousTarget;
  float duration;
  float tau;
  float timeElapsed;
} RobotAcceleration;

void RobotAcceleration_Init(RobotAcceleration *robot, float initialTarget, float duration);
void RobotAcceleration_SetTarget(RobotAcceleration *robot, float newTargetVelocity, float newDuration);
float RobotAcceleration_Update(RobotAcceleration *robot, float dt);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_ACCELERATION_H */

