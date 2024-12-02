#include "RotationStabilizer.h"
#include "helper_3dmath.h"

#include <cmath>


VectorFloat computeOmegaV(
    const Quaternion& firstQ,
    const Quaternion& secondQ,
    const float dt
)
{
    return VectorFloat{
        (2 / dt) * (secondQ.w * firstQ.x - secondQ.x * firstQ.w - secondQ.y * firstQ.z + secondQ.z * firstQ.y),
        (2 / dt) * (secondQ.w * firstQ.y + secondQ.x * firstQ.z - secondQ.y * firstQ.w - secondQ.z * firstQ.x),
        (2 / dt) * (secondQ.w * firstQ.z - secondQ.x * firstQ.y + secondQ.y * firstQ.x - secondQ.z * firstQ.w)
    };
}

VectorFloat computeStabilizingVector(
    const Quaternion& currentQ,
    const Quaternion& targetQ,
    Quaternion& previousQ,
    const float deltaTime,
    const float dampingThreshold,
    const float Kp,
    const float Ki,
    const float Kd,
    const float KiLimit
)
{
    const VectorFloat trueAV{ computeOmegaV(currentQ, previousQ, deltaTime) };
    const VectorFloat targetAV{ computeOmegaV(targetQ, currentQ, deltaTime * 2) };

    VectorFloat integralError{
    		(targetAV.x - trueAV.x) * deltaTime,
    		(targetAV.y - trueAV.y) * deltaTime,
    		(targetAV.z - trueAV.z) * deltaTime
    	};

    const float integralMagnitude = integralError.getMagnitude();

    if(integralMagnitude > KiLimit)
    {
    	integralError.x *= KiLimit / integralMagnitude;
		integralError.y *= KiLimit / integralMagnitude;
		integralError.z *= KiLimit / integralMagnitude;
	}

    VectorFloat angularVelocityAdjustment = {
        //Kp * (targetAV.x - trueAV.x) + Ki * integralError.x + Kd * (0 - trueAV.x),
        //Kp * (targetAV.y - trueAV.y) + Ki * integralError.y + Kd * (0 - trueAV.y),
        //Kp * (targetAV.z - trueAV.z) + Ki * integralError.z + Kd * (0 - trueAV.z)
        Kp * (targetAV.x - trueAV.x) + Ki * integralError.x + Kd * trueAV.x,
        Kp * (targetAV.y - trueAV.y) + Ki * integralError.y + Kd * trueAV.y,
        Kp * (targetAV.z - trueAV.z) + Ki * integralError.z + Kd * trueAV.z
    };

    // ugly but C++17 thing
    if (
        const float angleToTarget{
            std::acos(targetQ.w * currentQ.w +
                targetQ.x * currentQ.x +
                targetQ.y * currentQ.y +
                targetQ.z * currentQ.z)
        };
        angleToTarget < dampingThreshold
    ) {
        // P factor
        const float dampingFactor = angleToTarget / dampingThreshold;
        angularVelocityAdjustment.x *= dampingFactor;
        angularVelocityAdjustment.y *= dampingFactor;
        angularVelocityAdjustment.z *= dampingFactor;
    }

    previousQ = currentQ;

    return angularVelocityAdjustment;
}

Quaternion applyAngularVelocityToQuaternion(const Quaternion& qCurrent, const VectorFloat& angularVelocity, const float dt)
{
    const float omega = angularVelocity.getMagnitude();

    const float halfAngle = omega * dt / 2.0f;
    const float sinHalfAngle = std::sin(halfAngle);
    const float cosHalfAngle = std::cos(halfAngle);

    Quaternion omegaQuat = {
        cosHalfAngle,
        sinHalfAngle * (angularVelocity.x / omega),
        sinHalfAngle * (angularVelocity.y / omega),
        sinHalfAngle * (angularVelocity.z / omega)
    };

    const Quaternion updatedQuaternion = omegaQuat.getProduct(qCurrent);

    return updatedQuaternion.getNormalized();
}

Quaternion applyPitchRolltoQuaternion(const Quaternion& referenceQ,const int& PitchD, const int& RollD)
{

	double angleRadians = PitchD * 3.14159 / 180.0; // Convert degrees to radians
	double halfAngle = angleRadians / 2.0;
	Quaternion pitchQuaternion{ std::cos(halfAngle), 0.0, 0.0, std::sin(halfAngle) };

	angleRadians = RollD * 3.14159 / 180.0; // Convert degrees to radians
	halfAngle = angleRadians / 2.0;
	Quaternion rollQuaternion{ std::cos(halfAngle), std::sin(halfAngle), 0.0, 0.0 };

	Quaternion finish = referenceQ * pitchQuaternion * rollQuaternion;
	finish.normalize();
	return finish;

}