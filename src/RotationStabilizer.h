#pragma once

#ifndef _ROTATION_STABILIZER_H_
#define _ROTATION_STABILIZER_H_

#include "helper_3dmath.h"

VectorFloat computeOmegaV(
    const Quaternion& firstQ,
    const Quaternion& secondQ,
    const float dt
);


VectorFloat computeStabilizingVector(
    const Quaternion& currentQ,
    const Quaternion& targetQ,
    Quaternion& previousQ,
    const float deltaTime,
    const float dampingThreshold,
    const float Kp,
    const float Ki,
    const float Kd,
    const float KiLimit = 1.0f
);


Quaternion applyAngularVelocityToQuaternion(
    const Quaternion& qCurrent,
    const VectorFloat& angularVelocity,
    const float dt
);
Quaternion applyPitchRolltoQuaternion(
    const Quaternion& referenceQ,
    const int &PitchD,
    const int &RollD
);

#endif /* _ROTATION_STABILIZER_H_ */