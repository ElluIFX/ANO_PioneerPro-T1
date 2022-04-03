/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：飞控初始化
**********************************************************************************/
#include "include.h"
#include "Drv_pwm_out.h"
#include "Drv_led.h"
#include "Drv_spi.h"
#include "Drv_icm20602.h"
#include "drv_ak8975.h"
#include "drv_spl06.h"
#include "Drv_w25qxx.h"
#include "Drv_i2c_soft.h"
#include "drv_vl53l0x.h"
#include "Ano_FlightCtrl.h"
#include "Drv_adc.h"
#include "Drv_heating.h"
#include "Ano_RC.h"


u8 of_init_type;

u8 All_Init()
{
	NVIC_PriorityGroupConfig(NVIC_GROUP);		//中断优先级组别设置

	SysTick_Configuration(); 		//滴答时钟
	
	Delay_ms(100);					//延时
	Drv_LED_Init();					//LED功能初始化
	
	Flash_Init();             		//板载FLASH芯片驱动初始化
	
	Para_Data_Init();              		//参数数据初始化
	
//	PWM_IN_Init();					
	//初始化接收机采集功能
	Remote_Control_Init();
	
	PWM_Out_Init();					//初始化电调输出功能	
	Delay_ms(100);					//延时
	
	Drv_SPI2_init();          		//spi_2初始化，用于读取飞控板上所有传感器，都用SPI读取
	Drv_Icm20602CSPin_Init(); 		//spi片选初始化
	Drv_AK8975CSPin_Init();   		//spi片选初始化

	Drv_SPL06CSPin_Init();    		//spi片选初始化
	sens_hd_check.gyro_ok = sens_hd_check.acc_ok = 
	Drv_Icm20602Reg_Init();   		//icm陀螺仪加速度计初始化，若初始化成功，则将陀螺仪和加速度的初始化成功标志位赋值
	sens_hd_check.mag_ok = 1;       //标记罗盘OK	
	sens_hd_check.baro_ok = Drv_Spl0601_Init();       		//气压计初始化

	Usb_Hid_Init();					//飞控usb接口的hid初始化
	Delay_ms(100);					//延时
	
	Usart2_Init(500000);			//串口2初始化，函数参数为波特率
	Usart3_Init(500000);			//串口3初始化，函数参数为波特率
	Uart5_Init(500000);			//串口5初始化，函数参数为波特率
	Delay_ms(10);					//延时	
	Uart4_Init(500000);
	Delay_ms(10);					//延时

	
	I2c_Soft_Init();          		//软件i2c初始化，因为飞控可以外接匿名激光定高模块，需要保留一个IIC，读取外置传感器
	Drv_Vl53_Init();          		//TOF模块初始化，使用VL53L0X的激光测高，代替原有的超声波定高，效果更好

	Drv_AdcInit();
	Delay_ms(100);					//延时

	All_PID_Init();               		//PID初始化
////////////
	Drv_GpsPin_Init();				//GPS初始化 串口1
	Delay_ms(50);					//延时		
	
	Drv_HeatingInit();
//	Drv_HeatingSet(5);
	
	ANO_DT_SendString("SYS init OK!");
	return (1);
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
