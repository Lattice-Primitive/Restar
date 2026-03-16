/**
 * @file AHRS.h AHRS自动航向基准系统(Automatic Heading Reference System)
 * @author from internet
 * @note AHRS是自动航向基准系统(Automatic Heading Reference System)的简称。
 * 目前，使用四元数来进行AHRS姿态解算的算法被广泛采用于四轴飞行器上
*/
#ifndef __AHRS_H
#define __AHRS_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

typedef struct 
{
    float accel[3]; //测得加速度
    float gyro[3]; //测得角速度
    float magnet[3]; //测得磁场
} AHRS_IMU_Data_t;

typedef struct
{
    AHRS_IMU_Data_t data;//数据
    float Kp; //加速度计PID kp 典型值:10.0f 
    float Ki; //加速度计PID ki 典型值:0.008f
    float integralFB[3]; //积分值
    float sampleFreq; //取样频率
} AHRS_IMU_t;

typedef struct
{
    float q0;//四元数 w
    float q1;//四元数 x
    float q2;//四元数 y
    float q3;//四元数 z
} AHRS_Quaternions_Data_t;

typedef struct
{
    float x;//横滚角 roll
    float y;//俯仰角 pitch
    float z;//偏航角 yaw
}AHRS_EulerAngle_Data_t;

/**
 * @brief IMU结构体初始化
 * @param imu (AHRS_IMU_t*) IMU结构体指针
 * @param Kp (float) 加速度计PID kp 典型值:10.0f
 * @param Ki (float) 加速度计PID ki 典型值:0.008f
 * @param sampleFreq (float) 取样频率Hz
 * @return (void) NULL
*/
extern inline void AHRS_IMU_Init(
    AHRS_IMU_t *imu,
    float Kp,
    float Ki,
    float sampleFreq)
{
    imu->Kp=Kp;
    imu->Ki=Ki;
    imu->sampleFreq=sampleFreq;
    imu->integralFB[0]=0;
    imu->integralFB[1]=0;
    imu->integralFB[2]=0;
}

/**
 * @brief IMU新数据装填
 * @param imu (AHRS_IMU_t*) IMU结构体指针
 * @param data (AHRS_IMU_Data_t *) IMU数据结构体指针
 * @return (void) NULL
*/
extern inline void AHRS_IMU_GetData(
    AHRS_IMU_t *imu,
    AHRS_IMU_Data_t *data)
{
    imu->data.accel[0]=data->accel[0];
    imu->data.accel[1]=data->accel[1];
    imu->data.accel[2]=data->accel[2];

    imu->data.gyro[0]=data->gyro[0];
    imu->data.gyro[1]=data->gyro[1];
    imu->data.gyro[2]=data->gyro[2];

    imu->data.magnet[0]=data->magnet[0];
    imu->data.magnet[1]=data->magnet[1];
    imu->data.magnet[2]=data->magnet[2];
}

/**
 * @brief IMU快速获得偏航角，而无需将四元数转化为欧拉角
 * 要在AHRS_IMUdata2Quaternions_NoMagnet或AHRS_IMUdata2Quaternions后使用
 * @param imu (AHRS_IMU_t*) IMU结构体指针
 * @return (float) 偏航角
*/
extern inline float AHRS_IMU_GetFastYaw(
    AHRS_IMU_t *imu)
{
    return imu->integralFB[2];
}

/**
 * @brief 陀螺仪的加速度和角速度值转四元数，没有磁场数据
 * @param imu (AHRS_IMU_t *) IMU结构体指针
 * @param quat (AHRS_Quaternions_Data_t *)四元数结构体指针
 * @return (void) NULL
*/
extern void AHRS_IMUdata2Quaternions_NoMagnet(
    AHRS_IMU_t *imu,
    AHRS_Quaternions_Data_t *quat);

/**
 * @brief 陀螺仪的加速度、角速度、磁场值转四元数
 * @param imu (AHRS_IMU_t *) IMU结构体指针
 * @param quat (AHRS_Quaternions_Data_t *)四元数结构体指针
 * @return (void) NULL
*/
extern void AHRS_IMUdata2Quaternions(
    AHRS_IMU_t *imu,
    AHRS_Quaternions_Data_t *quat);
/**
 * @brief 四元数转欧拉角
 * @param quat (AHRS_Quaternions_Data_t *)四元数结构体指针
 * @param angle (AHRS_EulerAngle_Data_t *)欧拉角结构体指针
 * @return (void) NULL
*/
extern void AHRS_Quaternions2EulerAngle(
    AHRS_Quaternions_Data_t *quat,
    AHRS_EulerAngle_Data_t *angle);

/**
 * @brief 欧拉角转四元数
 * @param angle (AHRS_EulerAngle_Data_t *)欧拉角结构体指针
 * @param quat (AHRS_Quaternions_Data_t *)四元数结构体指针
 * @return (void) NULL
*/
extern void AHRS_EulerAngle2Quaternions(
    AHRS_EulerAngle_Data_t *angle,
    AHRS_Quaternions_Data_t *quat);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__AHRS_H*/
