#include "Correspond_Task.h"
#include "algorithm_SolveTrajectory.h"
#include "chassis_power_control.h"
#include "protocol_judgement.h"
#include "app_preference.h"
#include "tasks.h"
#include "Robot_Task.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include <vector>


extern uint16_t CRC_INIT;

/*发送数据*/
float set_bo[15];
Correspondence_ctrl Corres;
extern float filter_aim_yaw_target;

void Correspond_Task(void *argument)
{
	/* USER CODE BEGIN StartDefaultTask */
	Corres.Corres_Init();
	/* Infinite loop */
	for (;;)
	{
		Corres.Corres_Feedback();
		Corres.Corres_Send();

		xQueueSend(Message_Queue, &ID_Data[CorrespondenceData], 0);
		osDelay(Correspondence_Task_Control_Time);
	}
	/* USER CODE END StartDefaultTask */
}

void Correspondence_ctrl::Corres_Init(void)
{
	WS2812_INIT();
}

void Correspondence_ctrl::Corres_Send(void)
{
	  vofa_justfloat_output(set_bo, 7, &Serial3_Ctrl);
		VISUAL_SERIAL.sendData((uint8_t *)&visual_new_send_data, sizeof(visual_new_send_data));
		Corres_DWT_dt=DWT_GetDeltaT(&Corres_DWT_Count);
	//CAN_Cmd.SendData(&hfdcan2, CAN_CAP_SENT_ID, &SuperCapS, 8);//向超级电容管理模块发送超级电容数据
}

void Correspondence_ctrl::Corres_Feedback(void)
{
	  set_bo[0]=0;
		set_bo[1]=Message.Gyro.Yaw_real_angle;//Gimbal.Yaw.angle;
		set_bo[2]=Message.visual_receive_new_data.yaw.F;//Gimbal.Yaw.angle_set;
		set_bo[3]=filter_aim_yaw_target;//Gimbal.DM_Pitch.angle;
	
		set_bo[4]=Gimbal.DM_Pitch.angle;//Gimbal.DM_Pitch.angle_set;
		set_bo[5]=Gimbal.DM_Pitch.angle_set;
	
	  set_bo[6]=Message.robo->shoot_data.initial_speed;
	
	
	  visual_new_send_data.head1=0x4D;
    visual_new_send_data.head2=0x41;
    
    visual_new_send_data.mode = 0x01;
    
    visual_new_send_data.q[0] = Quarternoin[0];
    visual_new_send_data.q[1] = Quarternoin[1];
    visual_new_send_data.q[2] = Quarternoin[2];
    visual_new_send_data.q[3] = Quarternoin[3];
    visual_new_send_data.q[4] = Quarternoin[4];
    visual_new_send_data.q[5] = Quarternoin[5];
    visual_new_send_data.q[6] = Quarternoin[6];
    visual_new_send_data.q[7] = Quarternoin[7];

    visual_new_send_data.bullet_speed = Message.robo->shoot_data.initial_speed;

    uint16_t crc= Get_CRC16_Check_Sum((uint8_t *)&visual_new_send_data.head1,17-2,CRC_INIT);
    visual_new_send_data.crc16 = ((crc & 0xFF) << 8) | ((crc >> 8) & 0xFF);


	if (Message.robo->game_robot_state.power_management_chassis_output == 0 || Message.robo->power_heat_data.chassis_power < 1 || Message.robo->game_robot_state.power_management_gimbal_output == 0)
	{
		SuperCapS.enable = 0x00;
	}
	else if ((Message.SuperCapR.situation == CAP_CLOSE || Message.SuperCapR.situation == CAP_OPEN) && Message.robo->game_robot_state.robot_level >= 1)
	{
		SuperCapS.enable = 0xff;
	}
	else
	{
		SuperCapS.enable = 0x00;
	}
	if (Chassis.Mode == CHASSIS_NO_MOVE)
	{
		SuperCapS.enable = 0x00;
	}

	SuperCapS.mode = 0xFF;

	SuperCapS.power = (uint8_t)Message.robo->power_heat_data.chassis_power;
	SuperCapS.power_limit = (uint8_t)Message.robo->game_robot_state.chassis_power_limit;

	Statistic_Update(xTaskGetTickCount());
}
