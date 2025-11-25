#include "Robot_Task.h"
#include "tasks.h"
#include "arm_math.h"
#include <math.h>
#include <vector>
#include "algorithm_SolveTrajectory.h"
#include "bsp_dwt.h"
#include "my_kalman.h"

Gimbal_Ctrl Gimbal;

/***************************************退弹算法参数***********************************************/
uint8_t Rollback_Circle = 1 / 18; // 退弹圈数（拨弹盘圈数）
bool Is_StartRollBack = false;	  // 是否开启退弹
/***************************************退弹算法参数***********************************************/

/***************************************热量算法***********************************************/
// 冷却速率查询表
static const float COOLING_RATE_TABLE[] =
	{
		// 冷却优先型 (等级1-10)
		12.0f, 14.0f, 16.0f, 18.0f, 20.0f, 22.0f, 24.0f, 26.0f, 28.0f, 30.0f};
/***************************************热量算法***********************************************/

uint16_t anglesteering;
uint8_t speed_flag;
extern bool fire_control;
uint8_t aaaa;
uint16_t flag;
uint8_t up;
uint8_t down;
int8_t sign_ = -1;
int8_t down_angle;
extern float TOP_dir;

float angle_;

// 云台任务
void Gimbal_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	Gimbal.Gimbal_Init();
	/* Infinite loop */
	for (;;)
	{
		Gimbal.Behaviour_Mode();
		Gimbal.Feedback_Update();
		Gimbal.Control();

		if (Gimbal.Mode == GIMBAL_NO_MOVE)
		{
			CAN_Cmd.SendData(&CAN_Cmd.Fric, 0, 0, 0, 0);
			CAN_Cmd.SendData(&CAN_Cmd.Trigger, 0, 0, 0, 0);
			CAN_Cmd.SendData(&CAN_Cmd.Gimbal, 0, 0, 0, 0);
			CAN_Cmd.DM_MIT_SendData(&CAN_Cmd.Gimbal_DM_Pitch, 0, 0, 0, 0, Gimbal.G_compensation_out);
		}
		else
		{
			Gimbal.Control_loop();

			CAN_Cmd.SendData(&CAN_Cmd.Trigger, 0, 0, Gimbal.Trigger.give_current, 0);
			CAN_Cmd.SendData(&CAN_Cmd.Fric, 0, 0, Gimbal.Fric1.give_current, Gimbal.Fric2.give_current);
			CAN_Cmd.SendData(&CAN_Cmd.Gimbal, Gimbal.Yaw.give_current, 0, 0, 0);
			CAN_Cmd.DM_MIT_SendData(&CAN_Cmd.Gimbal_DM_Pitch, 0, 0, 0, 0, Gimbal.DM_Pitch.tor_set); // 含重力补偿
		}
		Gimbal.Stop_TickCount = DWT_GetTimeline_s(); // 从系统上电到当前时刻运行时间
		xQueueSend(Message_Queue, &ID_Data[GimbalData], 0);

		osDelay(GIMBAL_CONTROL_TIME);
	}
	/* USER CODE END StartDefaultTask */
}

// 初始化
float Q=0.1,R=1.5;
void Gimbal_Ctrl::Gimbal_Init(void)
{
	RC_Ptr = get_remote_control_point();
	Gimbal.Mode = GIMBAL_NO_MOVE;
	VisualKalmanCreate(&yaw_kalman, Q, R);//Q系统误差 测量误差R 0.1  2

	Gimbal.Kmg = 0.49;	// 重力补偿系数

	Yaw.gimbal_motor_measure = CAN_Cmd.Gimbal.Get_Motor_Measure_Pointer(0);
	DM_Pitch.gimbal_motor_measure = CAN_Cmd.Gimbal_DM_Pitch.Get_DM_Motor_Measure_Pointer();
	Trigger.gimbal_motor_measure = CAN_Cmd.Trigger.Get_Motor_Measure_Pointer(0);
	Fric1.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(2);
	Fric2.gimbal_motor_measure = CAN_Cmd.Fric.Get_Motor_Measure_Pointer(3);

	// 手动
	PID.Init(&Yaw.PositinPid, POSITION, YAW_POSITION_PID_KP, YAW_POSITION_PID_KI, YAW_POSITION_PID_KD, YAW_POSITION_PID_MAX_OUT, YAW_POSITION_PID_MAX_IOUT, YAW_POSITION_PID_BAND_I);
	PID.Init(&Yaw.SpeedPid, POSITION, YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT, YAW_SPEED_PID_BAND_I);
	PID.Init(&DM_Pitch.PositinPid, POSITION, DM_PITCH_POSITION_PID_KP, DM_PITCH_POSITION_PID_KI, DM_PITCH_POSITION_PID_KD, DM_PITCH_POSITION_PID_MAX_OUT, DM_PITCH_POSITION_PID_MAX_IOUT, DM_PITCH_POSITION_PID_BAND_I);
	PID.Init(&DM_Pitch.SpeedPid, POSITION, DM_PITCH_SPEED_PID_KP, DM_PITCH_SPEED_PID_KI, DM_PITCH_SPEED_PID_KD, DM_PITCH_SPEED_PID_MAX_OUT, DM_PITCH_SPEED_PID_MAX_IOUT, DM_PITCH_SPEED_PID_BAND_I);

	// 视觉
	PID.Init(&Yaw.FollowPositinPid, POSITION, YAW_FOLLOW_POSITION_PID_KP, YAW_FOLLOW_POSITION_PID_KI, YAW_FOLLOW_POSITION_PID_KD, YAW_FOLLOW_POSITION_PID_MAX_OUT, YAW_FOLLOW_POSITION_PID_MAX_IOUT, YAW_FOLLOW_POSITION_PID_BAND_I);
	PID.Init(&Yaw.FollowSpeedPid, POSITION, YAW_FOLLOW_SPEED_PID_KP, YAW_FOLLOW_SPEED_PID_KI, YAW_FOLLOW_SPEED_PID_KD, YAW_FOLLOW_SPEED_PID_MAX_OUT, YAW_FOLLOW_SPEED_PID_MAX_IOUT, YAW_FOLLOW_SPEED_PID_BAND_I);
	PID.Init(&DM_Pitch.Visual_PositinPid, POSITION, PITCH_FOLLOW_POSITION_PID_KP, PITCH_FOLLOW_POSITION_PID_KI, PITCH_FOLLOW_POSITION_PID_KD, PITCH_FOLLOW_POSITION_PID_MAX_OUT, PITCH_FOLLOW_POSITION_PID_MAX_IOUT, PITCH_FOLLOW_POSITION_PID_BAND_I);
	PID.Init(&DM_Pitch.Visual_SpeedPid, POSITION, PITCH_FOLLOW_SPEED_PID_KP, PITCH_FOLLOW_SPEED_PID_KI, PITCH_FOLLOW_SPEED_PID_KD, PITCH_FOLLOW_SPEED_PID_MAX_OUT, PITCH_FOLLOW_SPEED_PID_MAX_IOUT, PITCH_FOLLOW_SPEED_PID_BAND_I);

	// 能量机关
	PID.Init(&Yaw.EnergyPositinPid, POSITION, YAW_ENERGY_POSITION_PID_KP, YAW_ENERGY_POSITION_PID_KI, YAW_ENERGY_POSITION_PID_KD, YAW_ENERGY_POSITION_PID_MAX_OUT, YAW_ENERGY_POSITION_PID_MAX_IOUT, YAW_ENERGY_POSITION_PID_BAND_I);
	PID.Init(&Yaw.EnergySpeedPid, POSITION, YAW_ENERGY_SPEED_PID_KP, YAW_ENERGY_SPEED_PID_KI, YAW_ENERGY_SPEED_PID_KD, YAW_ENERGY_SPEED_PID_MAX_OUT, YAW_ENERGY_SPEED_PID_MAX_IOUT, YAW_ENERGY_SPEED_PID_BAND_I);
	PID.Init(&DM_Pitch.EnergyPositinPid, POSITION, PITCH_ENERGY_POSITION_PID_KP, PITCH_ENERGY_POSITION_PID_KI, PITCH_ENERGY_POSITION_PID_KD, PITCH_ENERGY_POSITION_PID_MAX_OUT, PITCH_ENERGY_POSITION_PID_MAX_IOUT, PITCH_ENERGY_POSITION_PID_BAND_I);
	PID.Init(&DM_Pitch.EnergySpeedPid, POSITION, PITCH_ENERGY_SPEED_PID_KP, PITCH_ENERGY_SPEED_PID_KI, PITCH_ENERGY_SPEED_PID_KD, PITCH_ENERGY_SPEED_PID_MAX_OUT, PITCH_ENERGY_SPEED_PID_MAX_IOUT, PITCH_ENERGY_SPEED_PID_BAND_I);

	// 摩擦轮
	PID.Init(&Fric1.SpeedPid, POSITION, FRIC1_SPEED_PID_KP, FRIC1_SPEED_PID_KI, FRIC1_SPEED_PID_KD, FRIC1_PID_MAX_OUT, FRIC1_PID_MAX_IOUT, FRIC1_PID_BAND_I);
	PID.Init(&Fric2.SpeedPid, POSITION, FRIC2_SPEED_PID_KP, FRIC2_SPEED_PID_KI, FRIC2_SPEED_PID_KD, FRIC2_PID_MAX_OUT, FRIC2_PID_MAX_IOUT, FRIC2_PID_BAND_I);

	// 拨弹轮
	PID.Init(&Trigger.PositinPid, POSITION, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD, TRIGGER_ANGLE_PID_MAX_OUT, TRIGGER_ANGLE_PID_MAX_IOUT, TRIGGER_ANGLE_PID_BAND_I);
	PID.Init(&Trigger.SpeedPid, POSITION, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD, TRIGGER_SPEED_PID_MAX_OUT, TRIGGER_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_BAND_I); /*  */

	Data.pitch_offset_ecd = GIMBAL_PITCH_OFFSET_RAD;
	Data.pitch_max_angle = GIMBAL_PITCH_MAX_ANGLE;
	Data.pitch_min_angle = GIMBAL_PITCH_MIN_ANGLE;
	Data.Trigger_offset_ecd = Trigger.gimbal_motor_measure->ecd;
	Data.Gear = 0;
	Data.Fric_Gear[0] = FRIC_GEAR_SET_1;
	Data.Fric_Gear[1] = FRIC_GEAR_SET_2;
	Data.Fric_Gear[2] = FRIC_GEAR_SET_3;
	Data.Shoot_Frequency_m[0] = TRIGGER_ONE_S_SHOOT_NUM1; // 6
	Data.Shoot_Frequency_m[1] = TRIGGER_ONE_S_SHOOT_NUM2; // 8
	Data.Shoot_Frequency_m[2] = 20.0f;					  // TRIGGER_ONE_S_SHOOT_NUM3;//12
	Data.Loading_open = LOADING_OPEN_DUTY;
	Data.Loading_close = LOADING_CLOSE_DUTY;
	Data.Fric_Set[2] = 6300; //   6400/25m/s左右

	Feedback_Update();
}

// 数据更新
void Gimbal_Ctrl::Feedback_Update(void)
{
	// 达妙使能，使能前：红灯  使能后：绿灯
	if (DM_Pitch.gimbal_motor_measure->state != 1)
	{
		CAN_Cmd.DM_Motor_clear_error(&CAN_Cmd.Gimbal_DM_Pitch);
		osDelay(1);
		CAN_Cmd.DM_Motor_Enable(&CAN_Cmd.Gimbal_DM_Pitch);
		osDelay(1);
	}

	Gimbal_DWT_dt = DWT_GetDeltaT(&Gimbal_DWT_Count); // 查看该函数执行前一次和后一次运行时间差

	G_compensation_out = Kmg * cos(Gimbal.DM_Pitch.angle * PI / 180.f); // 弧度 重力补偿

#ifdef Motor_Gyro
	DM_Pitch.angle = PITCH_MOTOR_REVERSE * motor_relative_ECD_to_angle(Pitch.gimbal_motor_measure->ecd, Data.pitch_offset_ecd);
	DM_Pitch.speed = PITCH_MOTOR_REVERSE * Pitch.gimbal_motor_measure->speed_rpm * 6 * 0.1;
	Yaw.speed =
		Yaw.angle =
// 记得处理达妙弧度转ecd
//  DM_Pitch.angle_RAD=Message .Dm_Pitch .Motor_pos;
//	DM_Yaw.angle = DM_Pitch.angle=Message.Gyro.Pitch
#endif
#ifdef MCU_Gyro
			Yaw.speed = Message.Gyro.Yaw_speed; // 速度
	Yaw.angle = Message.Gyro.Yaw_angle;			// 度

	DM_Pitch.angle = -Message.Gyro.Pitch_angle;
	DM_Pitch.speed = -Message.Gyro.Pitch_speed;
#endif

	Trigger.speed = Trigger.gimbal_motor_measure->speed_rpm;
	Fric1.speed = Fric1.gimbal_motor_measure->speed_rpm;
	Fric2.speed = Fric2.gimbal_motor_measure->speed_rpm;

	Trig.new_angle = motor_relative_ECD_to_angle(Trigger.gimbal_motor_measure->ecd, Data.Trigger_offset_ecd);
	if (Trig.new_angle - Trig.last_angle > 180)
	{
		Trig.cycle--;
	}
	else if (Trig.new_angle - Trig.last_angle < -180)
	{
		Trig.cycle++;
	}
	Trig.last_angle = Trig.new_angle;
	Trigger.angle = Trig.new_angle + Trig.cycle * 360.0f;

	Flags.AutoShoot_Flag = AUTOSHOOT;

	// 枪管热量控制 总热量-消耗的热量
	// 步兵每发弹丸10热量  一级每秒减12热量
	// Data.Shoot_Frequency弹频，每秒发射多少发弹丸
	Data.Shoot_Frequency = HeatManageMent_Adaptive();

	Statistic_Update(xTaskGetTickCount());
}

// 按键和拨杆控制
void Gimbal_Ctrl::Behaviour_Mode(void)
{
	if (switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		Flags.RC_Flag = false;
	else
		Flags.RC_Flag = true;

	// 按键控制
	if (Flags.RC_Flag == false)
	{
		if (RC.read_key(&RC.Key.C, single, true))
		{
			// 无力
			Mode = GIMBAL_NO_MOVE;
		}
		if (RC.read_key(&RC.Key.X, single, true) && Mode == GIMBAL_NO_MOVE)
		{
			// 启动后为人为控制
			Mode = GIMBAL_Normal;
		}
		// 视觉开关
		RC.read_key(&RC.Press.R, even, &Flags.Visual_Flag);
		// 能量机关开关
		RC.read_key(&RC.Key.E, single, &Flags.Energy_Flag);
		// 装弹开关
		RC.read_key(&RC.Key.R, single, &Flags.Loading_Flag);
		// 摩擦轮开关
		RC.read_key(&RC.Key.Q, single, &Flags.Fric_Flag);
		// 拨弹轮开关
		if (Flags.Fric_Flag == true && Flags.AutoShoot_Flag == true)
		{
			RC.read_key(&RC.Press.L, even, &Flags.Shoot_Flag); // 长按左键连续射击
		}
		else if (Flags.Fric_Flag == true && Flags.AutoShoot_Flag == false)
		{
			Flags.Shoot_Flag = RC.read_key(&RC.Press.L, single, true); // 左键单击，发射一枚
		}
		RC.read_key(&RC.Key.ctrl, even, &Flags.Shoot_Reversal_Flag);
	}

	// 拨杆控制
	if (Flags.RC_Flag == true)
	{
		if (switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			Mode = GIMBAL_NO_MOVE;
		}
#if RC_CONTRAL_MODE == 0
		if (switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 下 下
			Mode = GIMBAL_NO_MOVE;
		}
		else if (switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 下 中
			Mode = GIMBAL_Normal;
			Flags.Visual_Flag = false;
			Flags.Fric_Flag = false;
			Flags.Shoot_Flag = false;
		}
		else if (switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 下 上
			Mode = GIMBAL_Normal;
			Flags.Fric_Flag = true;
			Flags.Shoot_Flag = true;
			Flags.Visual_Flag = false;
		}
		else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 中 中 自瞄
			Mode = GIMBAL_Normal;
			Flags.Fric_Flag = true;
			Flags.Shoot_Flag = false;
			Flags.Visual_Flag = true;
		}
		else if (switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 上 中
			Mode = GIMBAL_Normal;
			Flags.Fric_Flag = false;
			Flags.Shoot_Flag = false;
			Flags.Visual_Flag = false;
		}
		else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 中 下
			Mode = GIMBAL_Normal;
			Flags.Fric_Flag = false;
			Flags.Shoot_Flag = false;
			Flags.Visual_Flag = false;
		}
		else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_RIGHT]))
		{
			// 中 上
			Mode = GIMBAL_Normal;
			Flags.Fric_Flag = false;
			Flags.Shoot_Flag = false;
			Flags.Visual_Flag = false;
		}
#elif RC_CONTRAL_MODE == 1
		// 面向检录
		if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_down(RC_Ptr->rc.s[CHANNEL_LEFT]))
		{
			Flags.Visual_Flag = false;
			Flags.Fric_Flag = false;
			Flags.Shoot_Flag = false;
			//		Flags.Fric_Flag = true;
			//		Flags.Shoot_Flag = true;//测试拨弹轮
			Flags.Loading_Flag = false;
		}
		else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_mid(RC_Ptr->rc.s[CHANNEL_LEFT]))
		{
			Flags.Visual_Flag = true;
			;
			// Flags.Fric_Flag = true;
			// Flags.Shoot_Flag = false;
			//		Flags.Shoot_Flag = true;
			// Flags.Shoot_Flag = true;
			Flags.Loading_Flag = false;
		}
		else if (switch_is_mid(RC_Ptr->rc.s[CHANNEL_RIGHT]) && switch_is_up(RC_Ptr->rc.s[CHANNEL_LEFT]))
		{
			//	Flags.Shoot_Flag = true;
			// Flags.Fric_Flag = true;

			// Flags.Visual_Flag = true;
			// Flags.Fric_Flag = true;
			//	Flags.Shoot_Flag = true;
			//	Flags.Loading_Flag = false;
		}
#endif
	}
	Flag_Behaviour_Control();
}

// 标志位行为设置
void Gimbal_Ctrl::Flag_Behaviour_Control()
{
	if (Message.robo->game_robot_state.robot_id != 0 && Message.robo->game_robot_state.current_HP == 0)
	{
		// 死亡处理
		Flags.Shoot_Flag = false;
		Flags.Visual_Flag = false;
		Flags.Energy_Flag = false;
		Mode = GIMBAL_NO_MOVE;

		Yaw.angle_set = Yaw.angle;
		DM_Pitch.angle_set = DM_Pitch.angle;
	}

	if (Flags.Fric_Flag == true)
	{
		Data.FricSpeedSet = Data.Fric_Set[2];
	}
	else
	{
		Data.FricSpeedSet = 0.0f;
		Flags.Shoot_Flag = false; // 这里导致摩擦轮开启，才能开启拨弹盘
	}
	Fric1.speed_set = -Data.FricSpeedSet;
	Fric2.speed_set = Data.FricSpeedSet;

	if (Flags.Visual_Flag == true && Mode != GIMBAL_NO_MOVE && (Message.visual_receive_new_data.mode == 1 || Message.visual_receive_new_data.mode == 2))
	{
		Mode = GIMBAL_AIM;
		if (Message.visual_receive_new_data.mode == 2)
		{
			Flags.Shoot_Flag = true;
		}
		else
		{
			Flags.Shoot_Flag = false;
		}
	}

	if (Flags.Shoot_Flag == true)
	{
		if (Flags.AutoShoot_Flag == true) // 连发
		{
			Trigger.speed_set = TRIGGER_MOTOR_REVERSE * Data.Shoot_Frequency * 60.0f / TRIGGER_ONCE_SHOOT_NUM * TRIGGER_REDUCTION_RATIO;
		}
		else // 单发
		{
			Trigger.angle_set += TRIGGER_REDUCTION_RATIO * TRIGGER_MOTOR_REVERSE * 360.0f / TRIGGER_ONCE_SHOOT_NUM;
		}
	}
	else
	{
		if (Flags.AutoShoot_Flag == true)
		{
			Trigger.speed_set = 0.0f;
		}
		if (Flags.Fric_Flag == false)
		{
			Trigger.angle_set = Trigger.angle;
		}
	}

	// 模式切换保护
	if (Mode == Last_Mode)
	{
		return;
	}
	else if (Mode != Last_Mode)
	{
		Yaw.angle_set = Yaw.angle;
		DM_Pitch.angle_set = DM_Pitch.angle;
		Trigger.angle_set = Trigger.angle;
		Last_Mode = Mode;
	}
}

// 摇杆和鼠标输入
void Gimbal_Ctrl::RC_to_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if (IsInvalid(*yaw_set) || IsInvalid(*pitch_set))
	{
		return;
	}
	// 遥控器原始通道值
	int16_t yaw_channel, pitch_channel;
	fp32 yaw_set_channel, pitch_set_channel;

	if (Flags.RC_Flag == true)
	{
		// 将遥控器的数据处理死区
		rc_deadline_limit(RC_Ptr->rc.ch[YawChannel], yaw_channel, RC_DEADLINE);
		rc_deadline_limit(RC_Ptr->rc.ch[PitchChannel], pitch_channel, RC_DEADLINE);

		yaw_set_channel = -(yaw_channel * Yaw_RC_SEN);
		pitch_set_channel = -pitch_channel * Pitch_RC_SEN;
	}
	else if (Flags.RC_Flag == false)
	{
		yaw_set_channel = -(RC_Ptr->mouse.x * Yaw_Mouse_SEN);
		pitch_set_channel = -(RC_Ptr->mouse.y * Pitch_Mouse_SEN);
	}

	*yaw_set = yaw_set_channel;
	*pitch_set = pitch_set_channel;
}

// 云台控制设定
float yaw_kalman_data;
float last_aim_target;;
float filter_aim_yaw_target;
float feedforward;
void Gimbal_Ctrl::Behaviour_Control(fp32 *yaw_set, fp32 *pitch_set)
{
	if (Mode == GIMBAL_NO_MOVE)
	{
		*yaw_set = 0;
		*pitch_set = 0;
	}
	else if (Mode == GIMBAL_Normal)
	{
		RC_to_Control(yaw_set, pitch_set);
	}
	else if (Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		filter_aim_yaw_target=0.1*Message.visual_receive_new_data.yaw.F+0.9*last_aim_target;
		
		//yaw_kalman_data  = VisualKalmanFilter(&yaw_kalman, Message.visual_receive_new_data.yaw.F);
		*yaw_set = Visual_Handle(Message.Gyro.Yaw_real_angle, filter_aim_yaw_target);
		*pitch_set = Message.visual_receive_new_data.pitch.F;
		
		feedforward=25*(filter_aim_yaw_target-last_aim_target);
		last_aim_target=filter_aim_yaw_target;
		
	}
	else if (Mode == GIMBAL_NAV)
	{
	}
}

// PID计算
void Gimbal_Ctrl::Control(void)
{

	fp32 yaw_set;
	fp32 pitch_set;

	Behaviour_Control(&yaw_set, &pitch_set);

	if (Mode == GIMBAL_NO_MOVE)
	{
		yaw_set = 0;
		pitch_set = 0;
		Fric1.speed_set = 0.0f;
		Fric2.speed_set = 0.0f;
	}
	else if (Mode == GIMBAL_Normal)
	{
		Yaw.angle_set += yaw_set;
		DM_Pitch.angle_set += pitch_set;
	}
	else if (Mode == GIMBAL_AIM || Mode == GIMBAL_ENERGY)
	{
		Yaw.angle_set = yaw_set;
		DM_Pitch.angle_set = pitch_set;
	}
	else if (Mode == GIMBAL_NAV)
	{
	}

	// 退弹算法
	// rollback(true,1/8);
	//	if(Trigger.gimbal_motor_measure->given_current > TRIGGER_BLOCKED_CURRENT && TRIGGER_MOTOR_REVERSE == -1)
	//	{ // 拨弹轮防堵转
	//		Trigger.angle_set = Trigger.angle_set - TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_ANGLE;
	//		Trigger.speed_set = -TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_SPEED;
	//	}
	//	else if(Trigger.gimbal_motor_measure->given_current < -TRIGGER_BLOCKED_CURRENT && TRIGGER_MOTOR_REVERSE == 1)
	//	{ // 拨弹轮防堵转
	//		Trigger.angle_set = Trigger.angle_set - TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_ANGLE;
	//		Trigger.speed_set = TRIGGER_MOTOR_REVERSE * TRIGGER_BLOCKED_SPEED;
	//	}

	DM_Pitch.angle_set = constrain(DM_Pitch.angle_set, Data.pitch_min_angle, Data.pitch_max_angle);
}

// 云台控制PID运算


void Gimbal_Ctrl::Control_loop(void)
{
	fp32 YAW_out = 0;
	fp32 PITCH_out = 0;
	G_compensation_out = Kmg * cos(Gimbal.DM_Pitch.angle * PI / 180.f);

	if (Mode == GIMBAL_Normal)
	{
		PID.Calc(&Yaw.PositinPid, Yaw.angle, Yaw.angle_set);
		PID.Calc(&Yaw.SpeedPid, Yaw.speed, Yaw.PositinPid.out);

		PID.Calc(&DM_Pitch.PositinPid, DM_Pitch.angle, DM_Pitch.angle_set);
		PID.Calc(&DM_Pitch.SpeedPid, DM_Pitch.speed, DM_Pitch.PositinPid.out);

		YAW_out = Yaw.SpeedPid.out;
		PITCH_out = forwardfeed_pitch(DM_Pitch.SpeedPid.out) + G_compensation_out;
	}
	else if (Mode == GIMBAL_AIM)
	{
		PID.Calc(&Yaw.FollowPositinPid, 0, Yaw.angle_set);
		PID.Calc(&Yaw.FollowSpeedPid, Yaw.speed, Yaw.FollowPositinPid.out);

		PID.Calc(&DM_Pitch.Visual_PositinPid, DM_Pitch.angle, DM_Pitch.angle_set);
		PID.Calc(&DM_Pitch.Visual_SpeedPid, DM_Pitch.speed, DM_Pitch.Visual_PositinPid.out);

		
		
		YAW_out = Yaw.FollowSpeedPid.out+feedforward;
		PITCH_out = DM_Pitch.Visual_SpeedPid.out + G_compensation_out;
	}
	else if (Mode == GIMBAL_ENERGY)
	{
		PID.Calc(&Yaw.EnergyPositinPid, Yaw.angle, Yaw.angle_set);
		PID.Calc(&Yaw.EnergySpeedPid, Yaw.speed, Yaw.EnergyPositinPid.out);

		PID.Calc(&DM_Pitch.EnergyPositinPid, DM_Pitch.angle, DM_Pitch.angle_set);
		PID.Calc(&DM_Pitch.EnergySpeedPid, DM_Pitch.speed, DM_Pitch.EnergyPositinPid.out);

		YAW_out = Yaw.EnergySpeedPid.out;
		PITCH_out = DM_Pitch.EnergySpeedPid.out - Kmg * cos(Gimbal.DM_Pitch.angle * PI / 180.f); // why
	}
	else if (Mode == GIMBAL_NAV)
	{
	}

	PID.Calc(&Fric1.SpeedPid, Fric1.speed, Fric1.speed_set);
	PID.Calc(&Fric2.SpeedPid, Fric2.speed, Fric2.speed_set);

	if (Flags.AutoShoot_Flag == true)
	{
		PID.Calc(&Trigger.SpeedPid, Trigger.speed, Trigger.speed_set);
	}

	Yaw.give_current = YAW_out; /*齿轮传动，力矩方向相方*/
	DM_Pitch.tor_set = PITCH_out;
	Fric1.give_current = Fric1.SpeedPid.out;
	Fric2.give_current = Fric2.SpeedPid.out;
	Trigger.give_current = Trigger.SpeedPid.out;
}

// 规整ECD后转化成角度DEG，范围±180
fp32 Gimbal_Ctrl::motor_relative_ECD_to_angle(uint16_t angle, uint16_t offset_ecd)
{
	fp32 relative_angle;
	int32_t relative_ecd = angle - offset_ecd;
	if (relative_ecd > Half_ecd_range)
	{
		relative_ecd -= ecd_range;
	}
	else if (relative_ecd < -Half_ecd_range)
	{
		relative_ecd += ecd_range;
	}
	relative_angle = relative_ecd * ECD_TO_DEG;
	return relative_angle;
}

void rc_key_v_fresh_Gimbal(RC_ctrl_t *RC)
{
	Gimbal.RC.rc_key_v_set(RC);
}

Gimbal_Ctrl *get_gimbal_ctrl_pointer(void)
{
	return &Gimbal;
}

/*
采样周期：T=0.001秒(代码一毫秒运行一次所以这里取)
转动惯量：J=1
摩擦系数： f=1
角速度/力矩：G(s)=1/(s+1)
前馈环节：Gf(s)=s+1;
输出：out=in'+in=(in-last_in)/T+in

*/
float last_in = 0;
float T = 1.0f;
float Gimbal_Ctrl::forwardfeed(float in)
{
	float out;
	out = (in - last_in) / T + in;
	last_in = in;
	return out;
}

float last_in_pitch = 0;
float Gimbal_Ctrl::forwardfeed_pitch(float in)
{
	float out;
	out = (in - last_in_pitch) / T + in;
	last_in_pitch = in;
	return out;
}

void Gimbal_Ctrl::process_angle(float visual_angle, float yaw_gyro, float *aim_angle)
{
	if (visual_angle - yaw_gyro >= 270)
		*aim_angle = (visual_angle - yaw_gyro - 360);

	else if (visual_angle - yaw_gyro <= -270)
		*aim_angle = (visual_angle + yaw_gyro + 180) - Yaw.angle;
	else
		*aim_angle = visual_angle - yaw_gyro;
}

void Gimbal_Ctrl::rollback(bool is_start_rollback, uint8_t circle)
{
	static bool Is_ShootState = false;
	static int16_t NowCurrent, LastCurrent, AvgCurrent;
	if (is_start_rollback == true)
	{
		switch (rollbackstate)
		{
		case NORMAL:
			NowCurrent = Trigger.gimbal_motor_measure->given_current;
			AvgCurrent = 0.01 * NowCurrent + 0.99 * LastCurrent;
			LastCurrent = NowCurrent;
			if (AvgCurrent < 7000 && Trigger.gimbal_motor_measure->speed_rpm > 3000)
				Is_ShootState = true;
			if (AvgCurrent > 8000 && Is_ShootState == true)
				rollbackstate = STOP;
			break;
		case STOP:
			Trigger.speed_set = 0;
			Is_ShootState = false;
			if (abs(Trigger.gimbal_motor_measure->speed_rpm <= 10))
			{
				Trigger.angle_set = Trigger.angle - 360 * 90 * circle; // 计算回退角度//开启双闭环
				Flags.AutoShoot_Flag = false;						   // 开启双闭环
				rollbackstate = ROLLBACK;
			}
			break;
		case ROLLBACK:
			if (abs(Trigger.angle_set - Trigger.angle) < 10)
			{
				rollbackstate = NORMAL;
				Flags.AutoShoot_Flag = true; // 开环
			}
			break;
		}
	}
}

// 视觉过零处理 范围±180
fp32 Gimbal_Ctrl::Visual_Handle(fp32 angle, fp32 Visual_angle)
{
	if (Visual_angle - angle > 180)
	{
		Visual_angle -= 360;
	}
	else if (Visual_angle - angle < -180)
	{
		Visual_angle += 360;
	};
	return Visual_angle - angle;
}

bool is_HighShootFrequency = true;
fp32 Gimbal_Ctrl::HeatManageMent_Adaptive()
{
	int current_level = Message.robo->game_robot_state.robot_level;
	float heat_limit = Message.robo->game_robot_state.shooter_barrel_heat_limit;
	float current_heat = Message.robo->power_heat_data.shooter_17mm_1_barrel_heat;
	float remaining_heat = heat_limit - current_heat;
	float heat_ratio = remaining_heat / heat_limit;

	
	float cooling_rate_per_sec = get_cooling_rate_by_level(current_level);// 根据等级获取冷却速率
	float balance_frequency = cooling_rate_per_sec / 10.0f;//计算出来的临界弹频

	// 安全系数
	float safety_factor = 1.0f;
	float safe_balance_frequency = balance_frequency * safety_factor;

	// 状态切换逻辑
	if (is_HighShootFrequency)
	{
		// 热量下降到阈值，切换到低频模式
		if (heat_ratio <= 0.25f)
		{
			is_HighShootFrequency = false;
			return 3.2;
		}
		
		return 20.0f;
	}
	else
	{
		// 当前是低频模式
		if (heat_ratio > 0.75f)
		{
			// 热量恢复到高位，切换高频模式
			is_HighShootFrequency = true;
			return 20.0f;
		}
	
		return 3.2;
	}
}

fp32 Gimbal_Ctrl::get_cooling_rate_by_level(int current_level)
{
	return COOLING_RATE_TABLE[current_level];
}