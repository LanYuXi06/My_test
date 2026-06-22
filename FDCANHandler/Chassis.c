#include "Chassis.h"
#include "My_PID.h"
#include "usart.h"
#include <math.h>
#include <stdlib.h>


volatile Swerve_Module_t SwerveModule[SWERVE_MODULE_NUM];
uint8_t ChassisTxBuf[16];
PID_t YawAnglePID[4];
PID_t YawSpeedPID[4];
PID_t WheelSpeedPID[4];
float yaw_target_deg = 0.0f;
float yaw_err_target[4];
float yaw_now=0.0f;
float yaw_err=0.0f;
float vx,vy,wz,wheel_vx,wheel_vy=0.0f;
//方向校正
int calibration_wheel[4]={-1,1,1,-1};

uint8_t chassis_stop;

void Chassis_Init(void)//初始化四个舵轮，全部置零，调控并初始化PID
{
    for (uint8_t i = 0; i < SWERVE_MODULE_NUM; i++)
    {
			//轮向数据
        SwerveModule[i].fb.wheel_angle_raw = 0;
        SwerveModule[i].fb.wheel_speed = 0;
        SwerveModule[i].fb.wheel_current = 0;
        SwerveModule[i].fb.wheel_temperature = 0;
      //航向数据
        SwerveModule[i].fb.yaw_angle_deg = 0.0f;
        SwerveModule[i].fb.yaw_speed = 0;
        SwerveModule[i].fb.yaw_current = 0;
        SwerveModule[i].fb.yaw_temperature = 0;

        SwerveModule[i].cmd.wheel_current_cmd = 0;
        SwerveModule[i].cmd.yaw_current_cmd = 0;
        SwerveModule[i].feedback_count = 0;
			
			//航向电机角度PID
				 WheelSpeedPID[i].Kp=10.0f;
				 WheelSpeedPID[i].Ki=0.1f;
				 WheelSpeedPID[i].Kd=0.2f;
         WheelSpeedPID[i].OutMax=3000.0f;
         WheelSpeedPID[i].OutMin=-3000.0f;
				 PID_Init(&WheelSpeedPID[i]);
			 //航向电机PID
			 YawSpeedPID[i].Kp=15.0f;
			 YawSpeedPID[i].Ki=0.4f;
			 YawSpeedPID[i].Kd=4.0f;
			 YawSpeedPID[i].OutMax=3000.0f;
			 YawSpeedPID[i].OutMin=-3000.0f;
			 PID_Init(&YawSpeedPID[i]);
			 //航向电机角度PID
			 YawAnglePID[i].Kp=10.0f;
			 YawAnglePID[i].Ki=0.0f;
			 YawAnglePID[i].Kd=0.0f;
			 YawAnglePID[i].OutMax=3000.0f;
			 YawAnglePID[i].OutMin=-3000.0f;
			 PID_Init(&YawAnglePID[i]);
    }

    for (uint8_t i = 0; i < 16; i++)
    {
        ChassisTxBuf[i] = 0;
    }

    SendChassisInstruction(ChassisTxBuf);//H7发送给G0
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)//G0反馈数据，H7解析数据
{
    FDCAN_RxHeaderTypeDef rxheader;
    uint8_t rx_buf[24] = {0};
    volatile Swerve_Module_t *module = 0;
    uint16_t yaw_raw = 0;
    
    //查看FIFO0是否接收到新数据
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    {
			return;//不是就退出
    }

    if (hfdcan->Instance != FDCAN1)//FDCAN是否是FDCAN1
    {
			return;//不是就退出
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxheader, rx_buf) != HAL_OK)
    {
        return;
    }

    if (rxheader.IdType != FDCAN_STANDARD_ID)
    {
        return;
    }

    switch (rxheader.Identifier)//判断是哪一个舵轮
    {
        case SteerLeftFrontId:
            module = &SwerveModule[SWERVE_LF];
            break;

        case SteerRightFrontId:
            module = &SwerveModule[SWERVE_RF];
            break;

        case SteerRightRearId:
            module = &SwerveModule[SWERVE_RR];
            break;

        case SteerLeftRearId:
            module = &SwerveModule[SWERVE_LR];
            break;

        default:
            return;
    }

    //数据合成，轮向电机数值
    module->fb.wheel_angle_raw = (int16_t)(((uint16_t)rx_buf[0] << 8) | rx_buf[1]);
    module->fb.wheel_speed = (int16_t)(((uint16_t)rx_buf[2] << 8) | rx_buf[3]);
    module->fb.wheel_current = (int16_t)(((uint16_t)rx_buf[4] << 8) | rx_buf[5]);
    module->fb.wheel_temperature = (int16_t)rx_buf[6];

    //航向电机角度，编码器数值
    yaw_raw = (uint16_t)(((uint16_t)rx_buf[8] << 8) | rx_buf[9]);
    module->fb.yaw_angle_deg = (float)yaw_raw / 65535.0f * 360.0f;//角度换算

		//将角度控制在-180~180
    if (module->fb.yaw_angle_deg > 180.0f)
    {
        module->fb.yaw_angle_deg -= 360.0f;
    }
		else if (module->fb.yaw_angle_deg < -180.0f)
    {
        module->fb.yaw_angle_deg += 360.0f;
    }
//数据合成，航向电机数值
    module->fb.yaw_speed = (int16_t)(((uint16_t)rx_buf[10] << 8) | rx_buf[11]);
    module->fb.yaw_current = (int16_t)(((uint16_t)rx_buf[12] << 8) | rx_buf[13]);
    module->fb.yaw_temperature = (int16_t)rx_buf[14];

    module->feedback_count++;
}

void Chassis_TestControlFromRemote(void)//将四个舵轮电流数据打包通过FDCAN1发送0X6FF 16字节给G0
{
    int32_t wheel_current = 0;
    int32_t yaw_current = 0;
    vx=RC.ch3*4;
	  vy=-RC.ch2*4;
	  wz=RC.ch4*4;
	chassis_stop=((RC.ch2 > -50 && RC.ch2 < 50) &&(RC.ch3 > -50 && RC.ch3 < 50) &&(RC.ch4 > -50 && RC.ch4 < 50));
	  float x[4]={HALF_LENGTH,HALF_LENGTH,-HALF_LENGTH,-HALF_LENGTH};
		float y[4]={HALF_LENGTH,-HALF_LENGTH,-HALF_LENGTH,HALF_LENGTH};
		float wheel_target[4];
		float angle_target[4];
    for (uint8_t i = 0; i < SWERVE_MODULE_NUM; i++)
    {
        SwerveModule[i].cmd.wheel_current_cmd = 0;
        SwerveModule[i].cmd.yaw_current_cmd = 0;
    }

    if (RC.s1 == SWERVE_TEST_ENABLE_SW)
    {
			//遥控器赋值比例乘4，传输成电流值
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			{
				//急停时可以保持原有的航向角度
				if(chassis_stop)
				{
				yaw_err_target[i]=0.0f;
				wheel_target[i]=0.0f;
					continue;
				}
				//舵轮底盘解算->平动速度+绕质心旋转W角速度
        wheel_vx = (int32_t)(vx-wz*y[i]);//x轴方向上的速度分量
        wheel_vy = (int32_t)(vy+wz*x[i]);//y轴方向上的速度分量
				wheel_target[i] = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy)*calibration_wheel[i];//wheel_vx和wheel_vy的和速度
        angle_target[i] = atan2f(wheel_vy, wheel_vx) * 57.29578f;//将弧度制转成角度
				//基础数据赋值
				yaw_now=SwerveModule[i].fb.yaw_angle_deg;
				yaw_err=angle_target[i]-yaw_now;
					//角度差控制在180~-180之间
						while(yaw_err > 180.0f)   yaw_err -= 360.0f;
            while(yaw_err < -180.0f)  yaw_err += 360.0f;
					//最短舵轮角度优化
					if(yaw_err>90.0f)
					{
					yaw_err-=180.0f;
					wheel_target[i]=-wheel_target[i];
					}
				  else if(yaw_err<-90.0f)
					{
					yaw_err+=180.0f;
						wheel_target[i]=-wheel_target[i];
					}
					yaw_err_target[i]=yaw_err;
			}
		//===========================此处进行PID调试===================// 
				
				//=================这里处理轮向速度环PID=========//
				for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
				{
				WheelSpeedPID[i].Target=wheel_target[i];
				WheelSpeedPID[i].Actual=SwerveModule[i].fb.wheel_speed;
				PID_Update(&WheelSpeedPID[i]);
        SwerveModule[i].cmd.wheel_current_cmd = (int16_t)WheelSpeedPID[i].Out;
				}
				
				//=================这里处理航向串级PID=========//
				for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
				{
				//外环角度环
				YawAnglePID[i].Target=yaw_err_target[i];
				YawAnglePID[i].Actual=0.0f;
				PID_Update(&YawAnglePID[i]);
				//内环速度环
				YawSpeedPID[i].Target=YawAnglePID[i].Out;
				YawSpeedPID[i].Actual=SwerveModule[i].fb.yaw_speed;
				PID_Update(&YawSpeedPID[i]);
        SwerveModule[i].cmd.yaw_current_cmd = (int16_t)YawSpeedPID[i].Out;
				}
    }
    //[0] [1]   第0组轮向电机电流
		//[2] [3]   第1组轮向电机电流
		//[4] [5]   第2组轮向电机电流
		//[6] [7]   第3组轮向电机电流

		//[8] [9]   第0组航向电机电流
		//[10][11]  第1组航向电机电流
		//[12][13]  第2组航向电机电流
		//[14][15]  第3组航向电机电流
    for (uint8_t i = 0; i < 16; i++)//清空当前数据，为后面传输出做铺垫
    {
        ChassisTxBuf[i] = 0;
    }

    for (uint8_t i = 0; i < SWERVE_MODULE_NUM; i++)
    {
        ChassisTxBuf[2 * i] =
            (uint8_t)(SwerveModule[i].cmd.wheel_current_cmd >> 8);//右移得到高八位

        ChassisTxBuf[2 * i + 1] =
            (uint8_t)(SwerveModule[i].cmd.wheel_current_cmd & 0xFF);//&处理得到低八位

        ChassisTxBuf[8 + 2 * i] =
            (uint8_t)(SwerveModule[i].cmd.yaw_current_cmd >> 8);

        ChassisTxBuf[8 + 2 * i + 1] =
            (uint8_t)(SwerveModule[i].cmd.yaw_current_cmd & 0xFF);
    }
    SendChassisInstruction(ChassisTxBuf);//H7传数据给G0,最重要的一个
}

void debug(void)
{
  uart_Printf("%.2f %.2f %.2f\n\r",
	WheelSpeedPID[0].Target,
	WheelSpeedPID[0].Actual,
	WheelSpeedPID[0].Out);
}
void PID_Command(char *ch)
{
if(ch[0]=='w')
	switch(ch[1])
	{
		case 'p':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			WheelSpeedPID[i].Kp=atof(&ch[2]);
		uart_Printf("wheelpid.kp=%.2f",WheelSpeedPID[0].Kp);
			break;
		case 'i':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			WheelSpeedPID[i].Ki=atof(&ch[2]);
		uart_Printf("wheelpid.ki=%.2f",WheelSpeedPID[0].Ki);
			break;
		case 'd':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			WheelSpeedPID[i].Kd=atof(&ch[2]);
		uart_Printf("wheelpid.kd=%.2f",WheelSpeedPID[0].Kd);
			break;
	}
	
	else if(ch[0]=='a')
		switch(ch[1])
	{
		case 'p':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			YawAnglePID[i].Kp=atof(&ch[2]);
		uart_Printf("wheelpid.kp=%.2f",YawAnglePID[0].Kp);
			break;
		case 'i':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			YawAnglePID[i].Ki=atof(&ch[2]);
		uart_Printf("wheelpid.ki=%.2f",YawAnglePID[0].Ki);
			break;
		case 'd':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			YawAnglePID[i].Kd=atof(&ch[2]);
		uart_Printf("wheelpid.kd=%.2f",YawAnglePID[0].Kd);
			break;
	}
	
	else if(ch[0]=='y')
		switch(ch[1])
	{
		case 'p':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			YawSpeedPID[i].Kp=atof(&ch[2]);
		uart_Printf("wheelpid.kp=%.2f",YawSpeedPID[0].Kp);
			break;
		case 'i':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			YawSpeedPID[i].Ki=atof(&ch[2]);
		uart_Printf("wheelpid.ki=%.2f",YawSpeedPID[0].Ki);
			break;
		case 'd':
			for(uint8_t i=0;i<SWERVE_MODULE_NUM;i++)
			YawSpeedPID[i].Kd=atof(&ch[2]);
		uart_Printf("wheelpid.kd=%.2f",YawSpeedPID[0].Kd);
			break;
	}
}
