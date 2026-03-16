/**
 * @file kalman_filter.h 卡尔曼滤波算法
 * @author from internet
 * @note 卡尔曼滤波算法，一种用于信号滤波的经典算法。
 * @link https://blog.csdn.net/lovebaby859450415/article/details/79082459?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522163117336616780255284545%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=163117336616780255284545&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-79082459.pc_search_ecpm_flag&utm_term=kalman+filter&spm=1018.2226.3001.4187
*/
#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#ifdef __cplusplus
extern "C"
{
#endif /*__cplusplus*/

typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
	float B;
    float Q;
    float R;
    float H;
}Kalman_t;

/**
 * @brief 卡尔曼滤波初始化
 * @param p (Kalman_t *) 卡尔曼滤波结构体
 * @param T_Q (float) 系统噪声协方差
 * @param T_R (float) 测量噪声协方差
 * @return (void) NULL
*/
void Kalman_Init(Kalman_t *p,float T_Q,float T_R);

/**
 * @brief 卡尔曼滤波运行
 * @param p (Kalman_t *) 卡尔曼滤波结构体
 * @param dat (float) 本次测量值
 * @return (float) 本次输出值
*/
float Kalman_Filter(Kalman_t* p,float dat);

#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__KALMAN_FILTER_H*/
