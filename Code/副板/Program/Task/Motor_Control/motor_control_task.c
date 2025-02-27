#include "motor_control_task.h"
#include "RemoteControl.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "Time7.h"
#include "function_task.h"
#include "gpio.h" 
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "BSP_MPU9250_Init.h"
#include "IMUTask.h"

#include "user_lib.h"
#include "math.h"
#include "stdlib.h"

/*==============================================================*/
#define MOTOR_CONTROL_TASK_PRIO 18
#define MOTOR_CONTROL_STK_SIZE 256
TaskHandle_t Motor_Control_Task_Handler;
void Motor_Control(void *pvParameters);

/*==============================================================*/
void task_Motor_Control_Create(void)
{
	xTaskCreate((TaskFunction_t)Motor_Control,
                (const char *)"Motor_Control",
                (uint16_t)MOTOR_CONTROL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)MOTOR_CONTROL_TASK_PRIO,
                (TaskHandle_t *)&Motor_Control_Task_Handler);
}
/*==============================================================*/

/*==============================================================*/
#define MODE_CHANGE_TASK_PRIO 18
#define MODE_CHANGE_STK_SIZE 256
TaskHandle_t Mode_Change_Task_Handler;
void Mode_Change(void *pvParameters);

/*==============================================================*/
void task_Mode_Change_Create(void)
{
	xTaskCreate((TaskFunction_t)Mode_Change,
                (const char *)"Mode_Change",
                (uint16_t)MODE_CHANGE_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)MODE_CHANGE_TASK_PRIO,
                (TaskHandle_t *)&Mode_Change_Task_Handler);
}
/*==============================================================*/


extern u32 time7_count;
extern u8 KEY_CAN;
extern remote_mode_e INPUTMOD;
fp32 up_age_pid[3] = {UP_ANGLE_PID_KP , UP_ANGLE_PID_KI , UP_ANGLE_PID_KD};
fp32 up_spd_pid[3] = {UP_SPEED_PID_KP , UP_SPEED_PID_KI , UP_SPEED_PID_KD};
fp32 clamp_age_pid[3] = {CLAMP_ANGLE_PID_KP , CLAMP_ANGLE_PID_KI , CLAMP_ANGLE_PID_KD};
fp32 clamp_spd_pid[3] = {CLAMP_SPEED_PID_KP , CLAMP_SPEED_PID_KI , CLAMP_SPEED_PID_KD};

Motor_Control_t motor_control;
/**
  * @brief  参数初始化
  * @param  电机结构体
  * @retval void
  * @attention 
  */
void Motor_Init(Motor_Control_t *motor_init)
{
	for(uint8_t i = 0; i < 2; i++)
	{
	  PID_Init(&motor_init->Up_motor_angle_pid[i] , PID_POSITION , up_age_pid , UP_ANGLE_PID_MAX_OUT , UP_ANGLE_PID_MAX_IOUT);
		PID_Init(&motor_init->Up_motor_speed_pid[i] , PID_POSITION , up_spd_pid , UP_SPEED_PID_MAX_OUT , UP_SPEED_PID_MAX_IOUT);
		motor_init->Up_motor[i].motor_measure = get_Up_Motor_Measure_Point(i);
		motor_init->Up_motor[i].angle_offset = 0.0f;
		motor_init->Up_motor[i].set_angle = motor_init->Up_motor[i].angle_offset;
	}
	PID_Init(&motor_init->Clamp_motor_angle_pid , PID_POSITION , clamp_age_pid , CLAMP_ANGLE_PID_MAX_OUT , CLAMP_ANGLE_PID_MAX_IOUT);
	PID_Init(&motor_init->Clamp_motor_speed_pid , PID_POSITION , clamp_spd_pid , CLAMP_SPEED_PID_MAX_OUT , CLAMP_SPEED_PID_MAX_IOUT);
	
	motor_init->mode = INIT;
	motor_init->motor_rc_ctrl = get_remote_control_point();
	motor_init->Clamp_motor.motor_measure = get_Clamp_Motor_Measure_Point();
	
	
}
/**
  * @brief  PID输出
  * @param  电机结构体
  * @retval void
  * @attention 
  */
void PID_OUTPUT(Motor_Control_t *Output_Calc)
{
		for(uint8_t i = 0; i < 2; i++)
			{
				PID_Calc(&Output_Calc->Up_motor_angle_pid[i] , Output_Calc->Up_motor[i].motor_measure->real_ecd , Output_Calc->Up_motor[i].set_angle);
				PID_Calc(&Output_Calc->Up_motor_speed_pid[i] , Output_Calc->Up_motor[i].motor_measure->speed_rpm , Output_Calc->Up_motor_angle_pid[i].out);
			}
		PID_Calc(&Output_Calc->Clamp_motor_angle_pid , Output_Calc->Clamp_motor.motor_measure->real_ecd , Output_Calc->Clamp_motor.set_angle);
		PID_Calc(&Output_Calc->Clamp_motor_speed_pid , Output_Calc->Clamp_motor.motor_measure->speed_rpm , Output_Calc->Clamp_motor_angle_pid.out);
			
//		Output_Calc->Clamp_motor_speed_pid.out = 0;
		CAN1_CMD_Control(Output_Calc->Up_motor_speed_pid[0].out , Output_Calc->Up_motor_speed_pid[1].out , Output_Calc->Clamp_motor_speed_pid.out ,0);
//		CAN1_CMD_Control(0,0,0,0);
}

/**
  * @brief  夹手转动到指定位置
  * @param  电机结构体
  * @retval void
  * @attention 
  */
void Clamp_Turn_To_Point(Motor_Control_t *Motor_Clamp_Turn, fp32 Location)
{
	TIM_Cmd(TIM7,ENABLE); //使能定时器7
	while(time7_count < TIME_STAMP_500MS)
	{
		Motor_Clamp_Turn->Clamp_motor.set_angle = Location;
		if(fabs(Location-Motor_Clamp_Turn->Clamp_motor.motor_measure->real_ecd)<0.1f)
		{
			TIM_Cmd(TIM7,DISABLE);
	    time7_count = 0;
			break;
		}
		
		PID_OUTPUT(Motor_Clamp_Turn);
		vTaskDelay(TIME_STAMP_1MS);//系统延时
	}
	TIM_Cmd(TIM7,DISABLE);
	time7_count = 0;
//	Motor_Clamp_Turn->Clamp_motor.set_angle = Location;
	
}

/**
  * @brief  架子运动到指定位置
  * @param  电机结构体, 左电机位置， 右电机位置
  * @retval void
  * @attention 
  */
void Up_Down_To_Point(Motor_Control_t *Motor_Up_Down, fp32 L_Location, fp32 R_Location)
{
	TIM_Cmd(TIM7,ENABLE); //使能定时器7
	while(time7_count < TIME_STAMP_500MS)
	{
		Motor_Up_Down->Up_motor[UP_RIGHT].set_angle = R_Location;
  	Motor_Up_Down->Up_motor[UP_LEFT].set_angle = L_Location;;
		if((fabs(L_Location-Motor_Up_Down->Up_motor[UP_LEFT].motor_measure->real_ecd)<0.1f) \
			&&(fabs(R_Location-Motor_Up_Down->Up_motor[UP_RIGHT].motor_measure->real_ecd)<0.1f))
		{
			TIM_Cmd(TIM7,DISABLE);
	    time7_count = 0;
			break;
		}
		PID_OUTPUT(Motor_Up_Down);
		vTaskDelay(TIME_STAMP_1MS);//系统延时
	}
	TIM_Cmd(TIM7,DISABLE);
	time7_count = 0;
//	Motor_Up_Down->Up_motor[UP_RIGHT].set_angle = R_Location;
//	Motor_Up_Down->Up_motor[UP_LEFT].set_angle = L_Location;
	
}
	u8 test_Flag = 0;
void Mode_Change(void *pvParameters)
{

	while(1)
	{
		if(motor_control.mode == INIT)
			{
				
				if(KEY_CAN == 1)//&& (Key1() == 1)&& (Flag == 0)
				{
					motor_control.Up_motor[0].set_angle += 10;
					motor_control.Up_motor[1].set_angle -= 10;
					
				  motor_control.Clamp_motor.set_angle = motor_control.Clamp_motor.motor_measure->real_ecd;
					test_Flag=1;
				}
				else if((KEY_CAN == 0) && (Key1() == 1) )//&& (Flag == 0)
				{
					for(uint8_t i = 0; i < 2; i++)
					{
					motor_control.Up_motor[i].angle_offset = motor_control.Up_motor[i].motor_measure->real_ecd;
					motor_control.Up_motor[i].set_angle = motor_control.Up_motor[i].angle_offset;
					}
					motor_control.Clamp_motor.angle_offset = (motor_control.Clamp_motor.motor_measure->real_ecd - 20.0f);
					motor_control.Clamp_motor.set_angle -= 5;
//					Flag = 1;
					test_Flag=2;
		      
				}
				else if((KEY_CAN == 0) && (Key1() == 0)) //&& (Flag == 1)
				{
//					motor_control.Clamp_motor.set_angle = motor_control.Clamp_motor.angle_offset;
				
					motor_control.mode = NORMAL;
//					Flag = 0;
					test_Flag=3;
				}
//				else 
//				{
//				  return;
//				}
//				if((KEY_CAN == 0) && (Key1() == 0) && (Flag == 0))
//				{
//					motor_control.mode = NORMAL;
//				}
//				if((IF_KEY_PRESSED_SHIFT) && (IF_KEY_PRESSED_Q))
//				{
//					if(IF_KEY_PRESSED_CTRL)
//					{
//						motor_control.mode = NORMAL;
//					}
//					else
//					{
//						motor_control.mode = TUNING;
//					}
//				}
//				if((RC_SW1_UP) && (RC_SW2_UP))
//				{
//					if(rc_ctrl.rc.ch[4] > 440)
//					{
//						motor_control.mode = NORMAL;
//					}
//					else
//					{
//						motor_control.mode = TUNING;
//					}
//				}
				
//				if(motor_control.mode == TUNING)
//				{
//					if(rc_ctrl.rc.ch[3] > 440)
//					{
//						motor_control.Up_motor[0].set_angle += 10;
//					  motor_control.Up_motor[1].set_angle -= 10;
//			
//						for(uint8_t i = 0; i < 2; i++)
//						{
//						motor_control.Up_motor[i].angle_offset = motor_control.Up_motor[i].motor_measure->real_ecd;
//						motor_control.Up_motor[i].set_angle = motor_control.Up_motor[i].angle_offset;
//						}
//						
//					}
//					if(rc_ctrl.rc.ch[1] > 440)
//					{
//						motor_control.Clamp_motor.set_angle -= 5;
//						
//						motor_control.Clamp_motor.angle_offset = motor_control.Clamp_motor.motor_measure->real_ecd;
//						motor_control.Clamp_motor.set_angle = motor_control.Clamp_motor.angle_offset;
//						
//						
//					}
//				}
			}
	vTaskDelay(1);
	}	
}

void Motor_Control(void *pvParameters)
{
	Motor_Init(&motor_control);
	while(1)
	{
		PID_OUTPUT(&motor_control);
		vTaskDelay(1);
	}
}
