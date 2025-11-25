#include "my_kalman.h"

/**
  * @name   VisualKalmanCreate
  * @brief  创建视觉数据卡尔曼滤波器
  * @param  p:  滤波器指针
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  * @retval none
  * @attention 针对视觉数据的卡尔曼滤波器，专门用于滤波视觉发送的yaw和pitch数据
  */
void VisualKalmanCreate(VisualKalman_t *p, float T_Q, float T_R)
{
  p->X_last = 0.0f;      // 上一时刻的最优结果
  p->P_last = 1.0f;      // 上一时刻最优结果的协方差
  p->Q = T_Q;            // 系统噪声协方差
  p->R = T_R;            // 测量噪声协方差
  p->A = 1.0f;           // 系统参数，假设为恒定速度模型
  p->B = 0.0f;           // 控制输入系数
  p->H = 1.0f;           // 观测矩阵
  p->X_mid = p->X_last;  // 当前时刻的预测结果
  p->P_mid = p->P_last;  // 当前时刻预测结果的协方差
}


/**
  * @name   VisualKalmanFilter
  * @brief  视觉数据卡尔曼滤波器
  * @param  p:  滤波器指针
  *         dat:待滤波的视觉数据
  * @retval 滤波后的数据
  * @attention 专门针对视觉数据的卡尔曼滤波，可以有效滤除视觉数据的噪声
  */
float VisualKalmanFilter(VisualKalman_t *p, float dat)
{
  // 预测步骤
  p->X_mid = p->A * p->X_last;                    // x(k|k-1) = A*X(k-1|k-1)
  p->P_mid = p->A * p->P_last * p->A + p->Q;      // p(k|k-1) = A*p(k-1|k-1)*A' + Q
  
  // 更新步骤
  p->kg = p->P_mid / (p->P_mid + p->R);           // kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H' + R)
  p->X_now = p->X_mid + p->kg * (dat - p->X_mid); // x(k|k) = X(k|k-1) + kg(k)*(Z(k)-H*X(k|k-1))
  p->P_now = (1 - p->kg) * p->P_mid;              // p(k|k) = (I-kg(k)*H)*P(k|k-1)
  
  // 状态更新
  p->P_last = p->P_now;
  p->X_last = p->X_now;
  
  return p->X_now; // 输出滤波结果
}


/**
 * 参数调试示例：
 * 
 * 情况1：视觉数据噪声较大，需要更强的滤波
 *   - 减小Q值（如0.01），增大R值（如2.0）
 *   - 效果：更平滑，但响应变慢
 * 
 * 情况2：视觉数据质量较好，需要快速响应
 *   - 增大Q值（如0.5），减小R值（如0.5）
 *   - 效果：响应更快，但可能有轻微噪声
 * 
 * 情况3：yaw和pitch轴特性不同
 *   - yaw轴：Q=0.1, R=1.0（通常需要更平滑）
 *   - pitch轴：Q=0.05, R=0.8（通常更稳定）
 */

/**
 * 调试技巧：
 * 
 * 1. 同时记录原始数据和滤波后数据，对比分析
 * 2. 观察云台控制的实际效果，是否出现抖动或延迟
 * 3. 根据比赛场景动态调整参数：
 *    - 远距离射击：需要更平滑的滤波
 *    - 近距离快速瞄准：需要更快的响应
 * 4. 考虑添加参数自适应功能，根据距离动态调整滤波强度
 */

// 参数自适应示例函数
void AdaptiveKalmanParameters(VisualKalman_t *kalman, float distance) 
{
    // 根据距离动态调整参数
    if (distance > 5.0f) {
        // 远距离：强滤波
        kalman->Q = 0.01f;
        kalman->R = 2.0f;
    } else if (distance > 2.0f) {
        // 中距离：中等滤波
        kalman->Q = 0.1f;
        kalman->R = 1.0f;
    } else {
        // 近距离：弱滤波，快速响应
        kalman->Q = 0.5f;
        kalman->R = 0.5f;
    }
}
