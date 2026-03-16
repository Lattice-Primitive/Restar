#include "AHRS.h"
#include <math.h>

#define invSqrt(value) (1.0f / sqrtf(value))

#define M_PI (3.14159265f)

void AHRS_IMUdata2Quaternions_NoMagnet(
    AHRS_IMU_t *imu,
    AHRS_Quaternions_Data_t *quat)
{
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float twoKp = 2 * imu->Kp;
    float twoKi = 2 * imu->Ki;
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((imu->data.accel[0] == 0.0f) && (imu->data.accel[1] == 0.0f) && (imu->data.accel[2] == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(imu->data.accel[0] * imu->data.accel[0] + imu->data.accel[1] * imu->data.accel[1] + imu->data.accel[2] * imu->data.accel[2]);
        imu->data.accel[0] *= recipNorm;
        imu->data.accel[1] *= recipNorm;
        imu->data.accel[2] *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = quat->q1 * quat->q3 - quat->q0 * quat->q2;
        halfvy = quat->q0 * quat->q1 + quat->q2 * quat->q3;
        halfvz = quat->q0 * quat->q0 - 0.5f + quat->q3 * quat->q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (imu->data.accel[1] * halfvz - imu->data.accel[2] * halfvy);
        halfey = (imu->data.accel[2] * halfvx - imu->data.accel[0] * halfvz);
        halfez = (imu->data.accel[0] * halfvy - imu->data.accel[1] * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            imu->integralFB[0] += twoKi * halfex * (1.0f / imu->sampleFreq); // integral error scaled by Ki
            imu->integralFB[1] += twoKi * halfey * (1.0f / imu->sampleFreq);
            imu->integralFB[2] += twoKi * halfez * (1.0f / imu->sampleFreq);
            imu->data.gyro[0] += imu->integralFB[0]; // apply integral feedback
            imu->data.gyro[1] += imu->integralFB[1];
            imu->data.gyro[2] += imu->integralFB[2];
        }
        else
        {
            imu->integralFB[0] = 0.0f; // prevent integral windup
            imu->integralFB[1] = 0.0f;
            imu->integralFB[2] = 0.0f;
        }

        // Apply proportional feedback
        imu->data.gyro[0] += twoKp * halfex;
        imu->data.gyro[1] += twoKp * halfey;
        imu->data.gyro[2] += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    imu->data.gyro[0] *= (0.5f * (1.0f / imu->sampleFreq)); // pre-multiply common factors
    imu->data.gyro[1] *= (0.5f * (1.0f / imu->sampleFreq));
    imu->data.gyro[2] *= (0.5f * (1.0f / imu->sampleFreq));
    qa = quat->q0;
    qb = quat->q1;
    qc = quat->q2;
    quat->q0 += (-qb * imu->data.gyro[0] - qc * imu->data.gyro[1] - quat->q3 * imu->data.gyro[2]);
    quat->q1 += (qa * imu->data.gyro[0] + qc * imu->data.gyro[2] - quat->q3 * imu->data.gyro[1]);
    quat->q2 += (qa * imu->data.gyro[1] - qb * imu->data.gyro[2] + quat->q3 * imu->data.gyro[0]);
    quat->q3 += (qa * imu->data.gyro[2] + qb * imu->data.gyro[1] - qc * imu->data.gyro[0]);

    // Normalise quaternion
    recipNorm = invSqrt(quat->q0 * quat->q0 + quat->q1 * quat->q1 + quat->q2 * quat->q2 + quat->q3 * quat->q3);
    quat->q0 *= recipNorm;
    quat->q1 *= recipNorm;
    quat->q2 *= recipNorm;
    quat->q3 *= recipNorm;
}

void AHRS_IMUdata2Quaternions(
    AHRS_IMU_t *imu,
    AHRS_Quaternions_Data_t *quat)
{
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    float twoKp = 2 * imu->Kp;
    float twoKi = 2 * imu->Ki;
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)

    if ((imu->data.magnet[0] == 0.0f) && (imu->data.magnet[1] == 0.0f) && (imu->data.magnet[2] == 0.0f))
    {

        AHRS_IMUdata2Quaternions_NoMagnet(imu, quat);
        return;
    }
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)

    if (!((imu->data.accel[0] == 0.0f) && (imu->data.accel[1] == 0.0f) && (imu->data.accel[2] == 0.0f)))
    {
        recipNorm = invSqrt(imu->data.accel[0] * imu->data.accel[0] + imu->data.accel[1] * imu->data.accel[1] + imu->data.accel[2] * imu->data.accel[2]);
        imu->data.accel[0] *= recipNorm;
        imu->data.accel[1] *= recipNorm;
        imu->data.accel[2] *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(imu->data.magnet[0] * imu->data.magnet[0] + imu->data.magnet[1] * imu->data.magnet[1] + imu->data.magnet[2] * imu->data.magnet[2]);
        imu->data.magnet[0] *= recipNorm;
        imu->data.magnet[1] *= recipNorm;
        imu->data.magnet[2] *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = quat->q0 * quat->q0;
        q0q1 = quat->q0 * quat->q1;
        q0q2 = quat->q0 * quat->q2;
        q0q3 = quat->q0 * quat->q3;
        q1q1 = quat->q1 * quat->q1;
        q1q2 = quat->q1 * quat->q2;
        q1q3 = quat->q1 * quat->q3;
        q2q2 = quat->q2 * quat->q2;
        q2q3 = quat->q2 * quat->q3;
        q3q3 = quat->q3 * quat->q3;

        // Reference direction of Earth’s magnetic field
        hx = 2.0f * (imu->data.magnet[0] * (0.5f - q2q2 - q3q3) + imu->data.magnet[1] * (q1q2 - q0q3) + imu->data.magnet[2] * (q1q3 + q0q2));
        hy = 2.0f * (imu->data.magnet[0] * (q1q2 + q0q3) + imu->data.magnet[1] * (0.5f - q1q1 - q3q3) + imu->data.magnet[2] * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (imu->data.magnet[0] * (q1q3 - q0q2) + imu->data.magnet[1] * (q2q3 + q0q1) + imu->data.magnet[2] * (0.5f - q1q1 - q2q2));

        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (imu->data.accel[1] * halfvz - imu->data.accel[2] * halfvy) + (imu->data.magnet[1] * halfwz - imu->data.magnet[2] * halfwy);
        halfey = (imu->data.accel[2] * halfvx - imu->data.accel[0] * halfvz) + (imu->data.magnet[2] * halfwx - imu->data.magnet[0] * halfwz);
        halfez = (imu->data.accel[0] * halfvy - imu->data.accel[1] * halfvx) + (imu->data.magnet[0] * halfwy - imu->data.magnet[1] * halfwx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f)
        {
            imu->integralFB[0] += twoKi * halfex * (1.0f / imu->sampleFreq); // integral error scaled by Ki
            imu->integralFB[1] += twoKi * halfey * (1.0f / imu->sampleFreq);
            imu->integralFB[2] += twoKi * halfez * (1.0f / imu->sampleFreq);
            imu->data.gyro[0] += imu->integralFB[0]; // apply integral feedback
            imu->data.gyro[1] += imu->integralFB[1];
            imu->data.gyro[2] += imu->integralFB[2];
        }
        else
        {
            imu->integralFB[0] = 0.0f; // prevent integral windup
            imu->integralFB[1] = 0.0f;
            imu->integralFB[2] = 0.0f;
        }

        // Apply proportional feedback PID
        imu->data.gyro[0] += twoKp * halfex;
        imu->data.gyro[1] += twoKp * halfey;
        imu->data.gyro[2] += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    imu->data.gyro[0] *= (0.5f * (1.0f / imu->sampleFreq)); // pre-multiply common factors
    imu->data.gyro[1] *= (0.5f * (1.0f / imu->sampleFreq));
    imu->data.gyro[2] *= (0.5f * (1.0f / imu->sampleFreq));
    qa = quat->q0;
    qb = quat->q1;
    qc = quat->q2;
    quat->q0 += (-qb * imu->data.gyro[0] - qc * imu->data.gyro[1] - quat->q3 * imu->data.gyro[2]);
    quat->q1 += (qa * imu->data.gyro[0] + qc * imu->data.gyro[2] - quat->q3 * imu->data.gyro[1]);
    quat->q2 += (qa * imu->data.gyro[1] - qb * imu->data.gyro[2] + quat->q3 * imu->data.gyro[0]);
    quat->q3 += (qa * imu->data.gyro[2] + qb * imu->data.gyro[1] - qc * imu->data.gyro[0]);

    // Normalise quaternion
    recipNorm = invSqrt(quat->q0 * quat->q0 + quat->q1 * quat->q1 + quat->q2 * quat->q2 + quat->q3 * quat->q3);
    quat->q0 *= recipNorm;
    quat->q1 *= recipNorm;
    quat->q2 *= recipNorm;
    quat->q3 *= recipNorm;
}

void AHRS_Quaternions2EulerAngle(
    AHRS_Quaternions_Data_t *quat,
    AHRS_EulerAngle_Data_t *angle)
{
    float sinr_cosp = 2 * (quat->q0 * quat->q1 + quat->q2 * quat->q3);
    float cosr_cosp = 1 - 2 * (quat->q1 * quat->q1 + quat->q2 * quat->q2);
    angle->x=atan2f(sinr_cosp,cosr_cosp);

    float sinp = 2 * (quat->q0 * quat->q2 - quat->q1 * quat->q3);
    if (fabsf(sinp) >= 1)
        angle->y = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angle->y = asinf(sinp);
    
    float siny_cosp = 2 * (quat->q0 * quat->q3 + quat->q1 * quat->q2);
    float cosy_cosp = 1 - 2 * (quat->q2 * quat->q2 + quat->q3 * quat->q3);
    angle->z=atan2f(siny_cosp, cosy_cosp);
}

void AHRS_EulerAngle2Quaternions(
    AHRS_EulerAngle_Data_t *angle,
    AHRS_Quaternions_Data_t *quat)
{
    float cy = cosf(angle->z * 0.5f);
    float sy = sinf(angle->z * 0.5f);
    float cp = cosf(angle->y * 0.5f);
    float sp = sinf(angle->y * 0.5f);
    float cr = cosf(angle->x * 0.5f);
    float sr = sinf(angle->x * 0.5f);

    quat->q0 = cy * cp * cr + sy * sp * sr;
    quat->q1 = cy * cp * sr - sy * sp * cr;
    quat->q2 = sy * cp * sr + cy * sp * cr;
    quat->q3 = sy * cp * cr - cy * sp * sr;
}
