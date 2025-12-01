#include "Message_Task.h"
#include "tasks.h"
#include "algorithm_SolveTrajectory.h"
#include "bsp_dwt.h"

Message_Ctrl Message;
extern Correspondence_ctrl Corres;

void Message_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	Message.Init();
	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(Message_Queue, &ID_Data[MessageData], portMAX_DELAY))
		{
			Guard.Feed(ID_Data[MessageData].Data_ID);
			Guard.Feed(MessageData);
			Message.Statistic_Update(xTaskGetTickCount());
		}
		if (RC_data_is_error(Message.RC_Ptr))
		{
			slove_RC_lost();
		}
	}
	/* USER CODE END StartDefaultTask */
}


void CAN1_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN1_Rx_Data;
	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(CAN1_Rx_Queue, &CAN1_Rx_Data, portMAX_DELAY))
		{
			Message.CAN1_Process((CanRxMsg *)CAN1_Rx_Data.Data_Ptr);
			Guard.Feed(CanData1);
		}
	}
	/* USER CODE END StartDefaultTask */
}


void CAN2_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN2_Rx_Data;
	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(CAN2_Rx_Queue, &CAN2_Rx_Data, portMAX_DELAY))
		{
			Message.CAN2_Process((CanRxMsg *)CAN2_Rx_Data.Data_Ptr);
			Guard.Feed(CanData2);
		}
	}
	/* USER CODE END StartDefaultTask */
}


void CAN3_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t CAN3_Rx_Data;
	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(CAN3_Rx_Queue, &CAN3_Rx_Data, portMAX_DELAY))
		{
			Message.CAN3_Process((CanRxMsg *)CAN3_Rx_Data.Data_Ptr);
			Guard.Feed(CanData3);
		}
	}
	/* USER CODE END StartDefaultTask */
}


void Serial_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t Serial_Rx_Data;
	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(Serial_Rx_Queue, &Serial_Rx_Data, portMAX_DELAY))
		{
			switch (Serial_Rx_Data.Data_ID)
			{
			case SerialData1:
				Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial1_Ctrl);
				break;
			case SerialData3:
				Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial3_Ctrl);
				break;
			case SerialData4:
				Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial4_Ctrl);
				break;
			case SerialData7:
				Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial7_Ctrl);
				break;
			case SerialData8:
				Message.Serialx_Hook((uint8_t *)Serial_Rx_Data.Data_Ptr, &Serial8_Ctrl);
				break;
			default:
				break;
			}
			Guard.Feed(Serial_Rx_Data.Data_ID);
		}
	}
	/* USER CODE END StartDefaultTask */
}


void Referee_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t Referee_Rx_Data;
	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(Referee_Rx_Queue, &Referee_Rx_Data.Data_Ptr, portMAX_DELAY))
		{
			referee_data_solve(&(((uint8_t *)Referee_Rx_Data.Data_Ptr)[1]));
			Guard.Feed(RefereeData);
		}
	}
	/* USER CODE END StartDefaultTask */
}


void DR16_Rx_Task(void *pvParameters)
{
	/* USER CODE BEGIN StartDefaultTask */
	static ID_Data_t DR16_Rx_Data;
	// 遥控器控制变量

	/* Infinite loop */
	for (;;)
	{
		if (xQueueReceive(DR16_Rx_Queue, &DR16_Rx_Data.Data_Ptr, portMAX_DELAY))
		{
			sbus_to_rc(&(((uint8_t *)(DR16_Rx_Data.Data_Ptr))[1]), Message.RC_Ptr);
			rc_key_v_fresh_Gimbal(Message.RC_Ptr);
			rc_key_v_fresh_Chassis(Message.RC_Ptr);
			Guard.Feed(RCData);
			if (RC_data_is_error(Message.RC_Ptr))
			{
				slove_data_error();
			}
		}
	}
	/* USER CODE END StartDefaultTask */
}


void Message_Ctrl::Init()
{
	Message.RC_Ptr = &RC_ctrl;
	CAN_ALL_Init();
	Prefence_Init();
	Serial_ALL_Init();
	robo = get_robo_data_Point();
}


void Message_Ctrl::Serialx_Hook(uint8_t *Rx_Message, Serialctrl *Serialx_Ctrl)
{
	if (Serialx_Ctrl == &VISUAL_SERIAL)
	{
		Visual_Serial_Hook(Rx_Message);
	}
	if (Serialx_Ctrl == &JUDGE_SERIAL)
	{
		xQueueSend(Referee_Rx_Queue, &Rx_Message, 0);
	}
	if (Serialx_Ctrl == &NAV_SERIAL)
	{
		NAV_Serial_Hook(Rx_Message);
	}
	if (Serialx_Ctrl == &DR16_SERIAL)
	{
		xQueueSend(DR16_Rx_Queue, &Rx_Message, 0);
	}
}




void Message_Ctrl::CAN1_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect = 0;
	uint8_t Len = 0;
	while (dect != 0xA5)
	{
		dect = CAN1_Ctrl.read();
		Len = CAN1_Ctrl.available();
		if (Len <= 12)
			return;
	}
	Len = CAN1_Ctrl.available();
	if (Len <= 12)
		return;
	dect = CAN1_Ctrl.read();
	if (dect != 0xA6)
		return;
	if (Len <= 11)
		return;
	for (uint8_t x = 0; x < 4; x++)
	{
		Rx_Data.StdId.u8[x] = CAN1_Ctrl.read();
	}
	for (uint8_t x = 0; x < 8; x++)
	{
		Rx_Data.Data[x] = CAN1_Ctrl.read();
	}

	switch (Rx_Data.StdId.u32)
	{
		#ifdef useHero
		case CAN_DJI_Motor3_ID:
		#endif
		case 0x010: // 四元数
		{
			Corres.Quarternoin[0] = Rx_Data.Data[0];
			Corres.Quarternoin[1] = Rx_Data.Data[1];
			Corres.Quarternoin[2] = Rx_Data.Data[2];
			Corres.Quarternoin[3] = Rx_Data.Data[3];
			Corres.Quarternoin[4] = Rx_Data.Data[4];
			Corres.Quarternoin[5] = Rx_Data.Data[5];
			Corres.Quarternoin[6] = Rx_Data.Data[6];
			Corres.Quarternoin[7] = Rx_Data.Data[7];
			break;
		}

		case 0x001: // 陀螺仪
		{
			
			CAN_MPU_R_XY.AngleY = ((int16_t)((Rx_Data.Data[1]) << 8 | (Rx_Data.Data[0]))) * FP32_MPU_RAD;
			CAN_MPU_R_XY.Speed_Y = ((int16_t)((Rx_Data.Data[3]) << 8 | (Rx_Data.Data[2]))) * BMI088_GYRO_2000_SEN;

			CAN_MPU_R_Z.AngleZ = ((int16_t)((Rx_Data.Data[5]) << 8 | (Rx_Data.Data[4]))) * FP32_MPU_RAD;
			CAN_MPU_R_Z.Speed_Z = ((int16_t)((Rx_Data.Data[7]) << 8 | (Rx_Data.Data[6]))) * BMI088_GYRO_2000_SEN;

			Gyro.Error_angle = (CAN_MPU_R_Z.AngleZ - Gyro.Last_angle);

			if (Gyro.Error_angle > 270.0f)
				Gyro.Yaw_cycle--;
			if (Gyro.Error_angle < -270.0f)
				Gyro.Yaw_cycle++;
			Gyro.Yaw_real_angle = CAN_MPU_R_Z.AngleZ;
			Gyro.Yaw_angle = CAN_MPU_R_Z.AngleZ + Gyro.Yaw_cycle * 360.0f;
			Gyro.Yaw_speed = CAN_MPU_R_Z.Speed_Z;
			Gyro.Last_angle = CAN_MPU_R_Z.AngleZ;

			Gyro.Pitch_angle = CAN_MPU_R_XY.AngleY;
			Gyro.Pitch_speed = CAN_MPU_R_XY.Speed_Y;
			
		
			break;
		}

		case 0x203: // 摩擦轮1
		{
			
			MA_get_motor_measure(CAN_Cmd.Fric.GetData(2), Rx_Data.Data);
			break;
		}
		case 0x204: // 摩擦轮2
		{
			MA_get_motor_measure(CAN_Cmd.Fric.GetData(3), Rx_Data.Data);
			break;
		}

		case DM_GimbalR_Pitch_ID: // 云台pitch
		{
			Get_DM_Motor_Measure(CAN_Cmd.Gimbal_DM_Pitch.GetData(), Rx_Data.Data, DM_4310_P_MIN, DM_4310_P_MAX, DM_4310_V_MIN, DM_4310_V_MAX, DM_4310_T_MIN, DM_4310_T_MAX);
			break;
		}

		default:
		{
			break;
		}
	}
}


void Message_Ctrl::CAN2_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect = 0;
	uint8_t Len = 0;
	while (dect != 0xA5)
	{
		dect = CAN2_Ctrl.read();
		Len = CAN2_Ctrl.available();
		if (Len <= 12)
			return;
	}
	Len = CAN2_Ctrl.available();
	if (Len <= 12)
		return;
	dect = CAN2_Ctrl.read();
	if (dect != 0xA6)
		return;
	if (Len <= 11)
		return;
	for (uint8_t x = 0; x < 4; x++)
	{
		Rx_Data.StdId.u8[x] = CAN2_Ctrl.read();
	}
	for (uint8_t x = 0; x < 8; x++)
	{
		Rx_Data.Data[x] = CAN2_Ctrl.read();
	}
	switch (Rx_Data.StdId.u32)
	{
		case CAN_DJI_Motor1_ID:
		case CAN_DJI_Motor2_ID:
		case CAN_DJI_Motor3_ID:
		case CAN_DJI_Motor4_ID: // 四个全向轮
		{
			static uint8_t i = 0;
			// 处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor1_ID;
			// 处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Chassis.GetData(i), Rx_Data.Data);
			break;
		}

		case CAN_DJI_Motor7_ID: // 拨弹盘
		{
			static uint8_t i = 0;
			// 处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor7_ID;
			// 处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Trigger.GetData(i), Rx_Data.Data);
			break;
		}

		case CAN_CAP_GET_ID: // 超级电容
		{
			
							//老超级电容
//            SuperCapR.situation = (uint8_t)(Rx_Data.Data[0]);
//            SuperCapR.mode = (uint8_t)(Rx_Data.Data[1]);
//            SuperCapR.power = (float)((uint16_t)((Rx_Data.Data[2]) | (Rx_Data.Data[3]) << 8)) * 0.1f;
//            SuperCapR.energy = (uint8_t)(Rx_Data.Data[4]);
//            SuperCapR.power_limit = (uint8_t)(Rx_Data.Data[5]);
//            SuperCapR.errorcode = (uint8_t)(Rx_Data.Data[6]);
			
			


						//新超电
            SuperCapR.situation = (uint8_t)(Rx_Data.Data[0]);
            SuperCapR.mode = (uint8_t)(Rx_Data.Data[1]);
            SuperCapR.power = (float)((uint16_t)((Rx_Data.Data[2]) | (Rx_Data.Data[3]) << 8)) * 0.1f;//
            SuperCapR.power_all = (float)((uint16_t)((Rx_Data.Data[4]) | (Rx_Data.Data[5]) << 8)) * 0.1f;//
            SuperCapR.energy = (uint8_t)(sqrt(Rx_Data.Data[6]*8.0f) + 0.01f*(SuperCapR.power_all-SuperCapR.power)/sqrt(Rx_Data.Data[6]*8.0f))*\
            (sqrt(Rx_Data.Data[6]*8.0f) + 0.01f*(SuperCapR.power_all-SuperCapR.power)/sqrt(Rx_Data.Data[6]*8.0f))/8.0f;
            SuperCapR.power_limit = (uint8_t)(Rx_Data.Data[7]);
            //SuperCapR.errorcode = (uint16_t)(Rx_Data.Data[8]);
            
			
			Guard.Feed(SupercapData);
			break;
		}

		default:
		{
			break;
		}
	}
}


void Message_Ctrl::CAN3_Process(CanRxMsg *Rx_Message)
{
	CanRxMsg Rx_Data;
	uint8_t dect = 0;
	uint8_t Len = 0;
	while (dect != 0xA5)
	{
		dect = CAN3_Ctrl.read();
		Len = CAN3_Ctrl.available();
		if (Len <= 12)
			return;
	}
	Len = CAN3_Ctrl.available();
	if (Len <= 12)
		return;
	dect = CAN3_Ctrl.read();
	if (dect != 0xA6)
		return;
	if (Len <= 11)
		return;
	for (uint8_t x = 0; x < 4; x++)
	{
		Rx_Data.StdId.u8[x] = CAN3_Ctrl.read();
	}
	for (uint8_t x = 0; x < 8; x++)
	{
		Rx_Data.Data[x] = CAN3_Ctrl.read();
	}
	
	
	switch (Rx_Data.StdId.u32)
	{
		case 0x205: // 云台yaw
		{
			MA_get_motor_measure(CAN_Cmd.Gimbal.GetData(0), Rx_Data.Data);
			break;
		}
		#ifdef useSteering
		case CAN_DJI_Motor5_ID:
		case CAN_DJI_Motor6_ID:
		case CAN_DJI_Motor7_ID:
		case CAN_DJI_Motor8_ID:
		{
			static uint8_t i = 0;
			// 处理电机ID号
			i = Rx_Data.StdId.u32 - CAN_DJI_Motor5_ID;
			// 处理电机数据宏函数
			MA_get_motor_measure(CAN_Cmd.Steer.GetData(i), Rx_Data.Data);
			break;
		}
		#endif
		default:
		{
			break;
		}
	}
}


void Message_Ctrl::NAV_Serial_Hook(uint8_t *Rx_Message)
{
	uint8_t len = Rx_Message[0];
	if (Rx_Message[1] != 0xAA)
		return;
	else
	{
		if (Rx_Message[2] == 0xA5)
		{
			MPU_DataZ.HHH = Rx_Message[1];
			MPU_DataZ.KEY = Rx_Message[2];
			MPU_DataZ.AngleZ.uint_8[0] = Rx_Message[3];
			MPU_DataZ.AngleZ.uint_8[1] = Rx_Message[4];
			MPU_DataZ.Speed_Z.uint_8[0] = Rx_Message[5];
			MPU_DataZ.Speed_Z.uint_8[1] = Rx_Message[6];
			MPU_DataZ.Acce_Z.uint_8[0] = Rx_Message[7];
			MPU_DataZ.Acce_Z.uint_8[1] = Rx_Message[8];
			MPU_DataZ.Acce_Y.uint_8[0] = Rx_Message[9];
			MPU_DataZ.Acce_Y.uint_8[1] = Rx_Message[10];
		}
		if (Rx_Message[2] == 0xA6)
		{
			MPU_DataXY.HHH = Rx_Message[1];
			MPU_DataXY.KEY = Rx_Message[2];
			MPU_DataXY.AngleX.uint_8[0] = Rx_Message[3];
			MPU_DataXY.AngleX.uint_8[1] = Rx_Message[4];
			MPU_DataXY.Speed_X.uint_8[0] = Rx_Message[5];
			MPU_DataXY.Speed_X.uint_8[1] = Rx_Message[6];
			MPU_DataXY.AngleY.uint_8[0] = Rx_Message[7];
			MPU_DataXY.AngleY.uint_8[1] = Rx_Message[8];
			MPU_DataXY.Speed_Y.uint_8[0] = Rx_Message[9];
			MPU_DataXY.Speed_Y.uint_8[1] = Rx_Message[10];
		}

		Gyro.Error_angle = (MPU_DataZ.AngleZ.int_16 * FP32_MPU_RAD - Gyro.Last_angle);
		if (Gyro.Error_angle > 270.0f)
			Gyro.Yaw_cycle--;
		if (Gyro.Error_angle < -270.0f)
			Gyro.Yaw_cycle++;
		Gyro.Yaw_real_angle = MPU_DataZ.AngleZ.int_16 * FP32_MPU_RAD; // 0到180-负180-0
		Gyro.Yaw_angle = MPU_DataZ.AngleZ.int_16 * FP32_MPU_RAD + Gyro.Yaw_cycle * 360.0f;
		Gyro.Yaw_speed = MPU_DataZ.Speed_Z.int_16 * BMI088_GYRO_2000_SEN;
		Gyro.Last_angle = MPU_DataZ.AngleZ.int_16 * FP32_MPU_RAD; // 0到180-负180-0

		// Gyro.Angle_X = MPU_DataXY.AngleX.int_16*FP32_MPU_RAD;
		// Gyro.Speed_X = MPU_DataXY.Speed_X.int_16*BMI088_GYRO_2000_SEN;
		Gyro.Pitch_angle = MPU_DataXY.AngleY.int_16 * FP32_MPU_RAD; // pitch
		Gyro.Pitch_speed = MPU_DataXY.Speed_Y.int_16 * BMI088_GYRO_2000_SEN;
	}
}


//视觉接收函数
void Message_Ctrl::Visual_Serial_Hook(uint8_t *Rx_Message)
{
	
	  visual_receive_new_data.len          = Rx_Message[0];
    if(visual_receive_new_data.len==13)
    {
        if(Verify_CRC16_Check_Sum(Rx_Message+1,13))
        {
            visual_receive_new_data.head1         = Rx_Message[1];
            visual_receive_new_data.head2         = Rx_Message[2];


            visual_receive_new_data.mode          = Rx_Message[3];

            visual_receive_new_data.yaw.I[0]      = Rx_Message[4];
            visual_receive_new_data.yaw.I[1]      = Rx_Message[5];
            visual_receive_new_data.yaw.I[2]      = Rx_Message[6];
            visual_receive_new_data.yaw.I[3]      = Rx_Message[7];

            visual_receive_new_data.pitch.I[0]    = Rx_Message[8];
            visual_receive_new_data.pitch.I[1]    = Rx_Message[9];
            visual_receive_new_data.pitch.I[2]    = Rx_Message[10];
            visual_receive_new_data.pitch.I[3]    = Rx_Message[11];

            visual_receive_new_data.crc16         = (Rx_Message[12]<<8) | Rx_Message[13];

            if(visual_receive_new_data.mode==0)
            {
                Gimbal.Flags.Recognized_target=false;
							  Gimbal.Flags.fire=false;
            }
            else if(visual_receive_new_data.mode==1)
            {
                Gimbal.Flags.Recognized_target=true;
							  Gimbal.Flags.fire=false;
            }
            else if(visual_receive_new_data.mode==2)
            {
                Gimbal.Flags.Recognized_target=true;
							  Gimbal.Flags.fire=true;
            }
						else
						{
								Gimbal.Flags.Recognized_target=false;
							  Gimbal.Flags.fire=false;
						}
        }
    }
		Messege_DWT_dt=DWT_GetDeltaT(&Messege_DWT_Count);
}


void Message_Ctrl::Gyro_CAN_Hook(uint32_t *Rx_Message, uint8_t *Rx_Data)
{

}



RC_ctrl_t *get_remote_control_point(void)
{
	return &RC_ctrl;
}


Message_Ctrl *get_message_ctrl_pointer(void)
{
	return &Message;
}


void Message_Ctrl::MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		return;
	}

	Gyro.calc.recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= Gyro.calc.recipNorm;
	ay *= Gyro.calc.recipNorm;
	az *= Gyro.calc.recipNorm;

	// Normalise magnetometer measurement
	// 将磁力计得到的实际磁场向量m单位化
	Gyro.calc.recipNorm = invSqrt(mx * mx + my * my + mz * mz);
	mx *= Gyro.calc.recipNorm;
	my *= Gyro.calc.recipNorm;
	mz *= Gyro.calc.recipNorm;

	// Auxiliary variables to avoid repeated arithmetic
	// 辅助变量，以避免重复运算
	Gyro.calc.q0q0 = Gyro.calc.q0 * Gyro.calc.q0;
	Gyro.calc.q0q1 = Gyro.calc.q0 * Gyro.calc.q1;
	Gyro.calc.q0q2 = Gyro.calc.q0 * Gyro.calc.q2;
	Gyro.calc.q0q3 = Gyro.calc.q0 * Gyro.calc.q3;
	Gyro.calc.q1q1 = Gyro.calc.q1 * Gyro.calc.q1;
	Gyro.calc.q1q2 = Gyro.calc.q1 * Gyro.calc.q2;
	Gyro.calc.q1q3 = Gyro.calc.q1 * Gyro.calc.q3;
	Gyro.calc.q2q2 = Gyro.calc.q2 * Gyro.calc.q2;
	Gyro.calc.q2q3 = Gyro.calc.q2 * Gyro.calc.q3;
	Gyro.calc.q3q3 = Gyro.calc.q3 * Gyro.calc.q3;

	// Reference direction of Earth's magnetic field
	// 通过磁力计测量值与坐标转换矩阵得到大地坐标系下的理论地磁向量
	Gyro.calc.hx = 2.0f * (mx * (0.5f - Gyro.calc.q2q2 - Gyro.calc.q3q3) + my * (Gyro.calc.q1q2 - Gyro.calc.q0q3) + mz * (Gyro.calc.q1q3 + Gyro.calc.q0q2));
	Gyro.calc.hy = 2.0f * (mx * (Gyro.calc.q1q2 + Gyro.calc.q0q3) + my * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q3q3) + mz * (Gyro.calc.q2q3 - Gyro.calc.q0q1));
	Gyro.calc.hz = 2.0f * (mx * (Gyro.calc.q1q3 - Gyro.calc.q0q2) + my * (Gyro.calc.q2q3 + Gyro.calc.q0q1) + mz * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q2q2));
	Gyro.calc.bx = sqrt(Gyro.calc.hx * Gyro.calc.hx + Gyro.calc.hy * Gyro.calc.hy);
	Gyro.calc.bz = 2.0f * (mx * (Gyro.calc.q1q3 - Gyro.calc.q0q2) + my * (Gyro.calc.q2q3 + Gyro.calc.q0q1) + mz * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q2q2));

	// Estimated direction of gravity and magnetic field
	// 将理论重力加速度向量与理论地磁向量变换至机体坐标系
	Gyro.calc.halfvx = Gyro.calc.q1q3 - Gyro.calc.q0q2;
	Gyro.calc.halfvy = Gyro.calc.q0q1 + Gyro.calc.q2q3;
	Gyro.calc.halfvz = Gyro.calc.q0q0 - 0.5f + Gyro.calc.q3q3;
	Gyro.calc.halfwx = Gyro.calc.bx * (0.5f - Gyro.calc.q2q2 - Gyro.calc.q3q3) + Gyro.calc.bz * (Gyro.calc.q1q3 - Gyro.calc.q0q2);
	Gyro.calc.halfwy = Gyro.calc.bx * (Gyro.calc.q1q2 - Gyro.calc.q0q3) + Gyro.calc.bz * (Gyro.calc.q0q1 + Gyro.calc.q2q3);
	Gyro.calc.halfwz = Gyro.calc.bx * (Gyro.calc.q0q2 + Gyro.calc.q1q3) + Gyro.calc.bz * (0.5f - Gyro.calc.q1q1 - Gyro.calc.q2q2);

	// Error is sum of cross product between estimated direction and measured direction of field vectors
	// 通过向量外积得到重力加速度向量和地磁向量的实际值与测量值之间误差
	Gyro.calc.halfex = (ay * Gyro.calc.halfvz - az * Gyro.calc.halfvy) + (my * Gyro.calc.halfwz - mz * Gyro.calc.halfwy);
	Gyro.calc.halfey = (az * Gyro.calc.halfvx - ax * Gyro.calc.halfvz) + (mz * Gyro.calc.halfwx - mx * Gyro.calc.halfwz);
	Gyro.calc.halfez = (ax * Gyro.calc.halfvy - ay * Gyro.calc.halfvx) + (mx * Gyro.calc.halfwy - my * Gyro.calc.halfwx);

	// Compute and apply integral feedback if enabled
	// 在PI补偿器中积分项使能情况下计算并应用积分项
	if (Gyro.calc.twoKi > 0.0f)
	{
		// integral error scaled by Ki
		// 积分过程
		Gyro.calc.integralFBx += Gyro.calc.twoKi * Gyro.calc.halfex * (1.0f / sampleFreq);
		Gyro.calc.integralFBy += Gyro.calc.twoKi * Gyro.calc.halfey * (1.0f / sampleFreq);
		Gyro.calc.integralFBz += Gyro.calc.twoKi * Gyro.calc.halfez * (1.0f / sampleFreq);

		// apply integral feedback
		// 应用误差补偿中的积分项
		gx += Gyro.calc.integralFBx;
		gy += Gyro.calc.integralFBy;
		gz += Gyro.calc.integralFBz;
	}
	else
	{
		// prevent integral windup
		// 避免为负值的Ki时积分异常饱和
		Gyro.calc.integralFBx = 0.0f;
		Gyro.calc.integralFBy = 0.0f;
		Gyro.calc.integralFBz = 0.0f;
	}

	// Apply proportional feedback
	// 应用误差补偿中的比例项
	gx += Gyro.calc.twoKp * Gyro.calc.halfex;
	gy += Gyro.calc.twoKp * Gyro.calc.halfey;
	gz += Gyro.calc.twoKp * Gyro.calc.halfez;

	// Integrate rate of change of quaternion
	// 微分方程迭代求解
	gx *= (0.5f * (1.0f / sampleFreq)); // pre-multiply common factors
	gy *= (0.5f * (1.0f / sampleFreq));
	gz *= (0.5f * (1.0f / sampleFreq));
	Gyro.calc.qa = Gyro.calc.q0;
	Gyro.calc.qb = Gyro.calc.q1;
	Gyro.calc.qc = Gyro.calc.q2;
	Gyro.calc.q0 += (-Gyro.calc.qb * gx - Gyro.calc.qc * gy - Gyro.calc.q3 * gz);
	Gyro.calc.q1 += (Gyro.calc.qa * gx + Gyro.calc.qc * gz - Gyro.calc.q3 * gy);
	Gyro.calc.q2 += (Gyro.calc.qa * gy - Gyro.calc.qb * gz + Gyro.calc.q3 * gx);
	Gyro.calc.q3 += (Gyro.calc.qa * gz + Gyro.calc.qb * gy - Gyro.calc.qc * gx);

	// Normalise quaternion
	// 单位化四元数 保证四元数在迭代过程中保持单位性质
	Gyro.calc.recipNorm = invSqrt(Gyro.calc.q0 * Gyro.calc.q0 + Gyro.calc.q1 * Gyro.calc.q1 + Gyro.calc.q2 * Gyro.calc.q2 + Gyro.calc.q3 * Gyro.calc.q3);
	Gyro.calc.q0 *= Gyro.calc.recipNorm;
	Gyro.calc.q1 *= Gyro.calc.recipNorm;
	Gyro.calc.q2 *= Gyro.calc.recipNorm;
	Gyro.calc.q3 *= Gyro.calc.recipNorm;
}

void Message_Ctrl::ToEulerAngles(fp32 q0, fp32 q1, fp32 q2, fp32 q3)
{
	Gyro.angle.AngleR_Calc = atan2f(2 * (q2 * q3 + q0 * q1), sq(q0) - sq(q1) - sq(q2) + sq(q3));
	Gyro.angle.AngleP_Calc = asinf(-2 * (q1 * q3 - q0 * q2));
	Gyro.angle.AngleY_Calc = atan2f(2 * (q1 * q2 + q0 * q3), sq(q0) + sq(q1) - sq(q2) - sq(q3));

	Gyro.angle.AngleR_Calc = degrees(Gyro.angle.AngleR_Calc);
	Gyro.angle.AngleP_Calc = degrees(Gyro.angle.AngleP_Calc);
	Gyro.angle.AngleY_Calc = degrees(Gyro.angle.AngleY_Calc);
}


float Message_Ctrl::Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2; // 计算速度

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if (S->delay_cnt > 300 /*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0; // 时间过长则认为速度不变
	}

	return S->processed_speed; // 计算出的速度
}


// 统计按键 按下次数：eg:  按下-松开  按下-松开  2次
// key_num==1代表有键盘按下
// key_num==0代表键盘松开
void rc_key_c::sum_key_count(int16_t key_num, count_num_key *temp_count)
{
	if (key_num == 1 && temp_count->key_flag == 0)
	{
		temp_count->key_flag = 1;
	}
	if (temp_count->key_flag == 1 && key_num == 0)
	{
		temp_count->count++;
		temp_count->key_flag = 0;
	}
}


void rc_key_c::clear_key_count(count_num_key *temp_count)
{
	temp_count->count = 0;
	temp_count->key_flag = 0;
}


// 按键单点赋值
bool rc_key_c::read_key_single(count_num_key *temp_count, bool *temp_bool)
{
	if ((temp_count->count >= 1) && *temp_bool == 0)
	{
		temp_count->count = 0;
		*temp_bool = true;
	}
	else if ((temp_count->count >= 1) && *temp_bool == 1)
	{
		temp_count->count = 0;
		*temp_bool = false;
	}
	return *temp_bool;
}


// 按键单点
bool rc_key_c::read_key_single(count_num_key *temp_count)
{
	if (temp_count->count >= 1)
	{
		temp_count->count = 0;
		return true;
	}
	else
	{
		temp_count->count = 0;
		return false;
	}
}


// 按键长按赋值q
bool rc_key_c::read_key_even(count_num_key *temp_count, bool *temp_bool)
{
	if (temp_count->key_flag == 1)
	{
		*temp_bool = true;
	}
	else if (temp_count->key_flag == 0)
	{
		*temp_bool = false;
	}
	return *temp_bool;
}


// 按键长按
bool rc_key_c::read_key_even(count_num_key *temp_count)
{
	if (temp_count->key_flag == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
}

uint8_t rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool clear)
{
	uint8_t result;
	if (clear == true)
	{
		if (mode == single)
		{
			result = read_key_single(temp_count);
		}
		else if (mode == even)
		{
			result = read_key_even(temp_count);
		}
	}
	else
	{
		if (mode == single)
		{
			result = temp_count->count;
		}
		else if (mode == even)
		{
			result = temp_count->key_flag;
		}
	}
	return result;
}


bool rc_key_c::read_key(count_num_key *temp_count, key_count_e mode, bool *temp_bool)
{
	if (mode == single)
	{
		read_key_single(temp_count, temp_bool);
	}
	else if (mode == even)
	{
		read_key_even(temp_count, temp_bool);
	}
	return *temp_bool;
}


// 更新按键
void rc_key_c::rc_key_v_set(RC_ctrl_t *RC)
{
	count_num_key *p = &Key.W;
	for (uint8_t i = 0; i < 16; i++)
	{
		if (RC->key.v & ((uint16_t)1 << i))
		{
			sum_key_count(1, (p + i));
		}
		else
		{
			sum_key_count(0, (p + i));
		}
	}
	// 鼠标
	if (RC->mouse.press_l == 1)
	{
		sum_key_count(1, &Press.L);
	}
	else
	{
		sum_key_count(0, &Press.L);
	}
	if (RC->mouse.press_r == 1)
	{
		sum_key_count(1, &Press.R);
	}
	else
	{
		sum_key_count(0, &Press.R);
	}
}
