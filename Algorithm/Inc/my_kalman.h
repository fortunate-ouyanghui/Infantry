#ifndef __MY_KALMAN_H
#define __MY_KALMAN_H

#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
	头文件缺少C++兼容性声明。当C++文件包含C头文件时，需要使用extern "C"包装。
*/

// 一维视觉卡尔曼滤波器结构体
typedef struct
{
    float X_last;   // 上一时刻的最优结果 x(k-1|k-1)
    float X_mid;    // 当前时刻的预测结果 x(k|k-1)
    float X_now;    // 当前时刻的最优结果 x(k|k)
    float P_mid;    // 当前时刻预测结果的协方差 P(k|k-1)
    float P_now;    // 当前时刻最优结果的协方差 P(k|k)
    float P_last;   // 上一时刻最优结果的协方差 P(k-1|k-1)
    float kg;       // 卡尔曼增益 K(k)
    float A;        // 状态转移矩阵（系统参数）
    float B;        // 控制输入系数
    float Q;        // 过程噪声协方差
    float R;        // 测量噪声协方差
    float H;        // 观测矩阵
} VisualKalman_t;


void VisualKalmanCreate(VisualKalman_t *p, float T_Q, float T_R);
float VisualKalmanFilter(VisualKalman_t *p, float dat);
void AdaptiveKalmanParameters(VisualKalman_t *kalman, float distance);

#ifdef __cplusplus
}
#endif

#endif
