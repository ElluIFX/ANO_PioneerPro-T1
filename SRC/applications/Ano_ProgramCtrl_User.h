#ifndef __ANO_PROGRAMCTRL_USER_H
#define __ANO_PROGRAMCTRL_USER_H

//==引用
//#include "sysconfig.h"
#include "Ano_FcData.h"

#define MAX_PC_XYVEL_CMPS 200
#define MAX_PC_ZVEL_CMPS 150
#define MAX_PC_PAL_DPS 100

void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps);
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps);
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps);

void User_Ctrl(u32 dT_ms);  //

//==定义
typedef struct {
  //
  float vel_cmps_set_h[2];
  float vel_cmps_set_w[2];
  float vel_cmps_set_ref[2];
  //
  float vel_cmps_set_z;
  float pal_dps_set;
} _pc_user_st;
extern _pc_user_st pc_user;

//==数据声明

typedef struct {
  u8 takeoff_en;
  u8 mode;
  u8 break_out;  //专门用于结束当前任务
  u8 stop;       //专门用于结束当前任务
  u8 land_en;
  u8 user_task_running;
  u8 user_task_end;
  u8 turn_left_en;
  u8 turn_right_en;
  u8 rc_lock;  //锁定遥控器
  float hx;
  float hy;
  float yaw;
  float z;
  u8 engage_hxy;
  u8 engage_yaw;
  u8 engage_z;
} _user_cntrl_word;

typedef struct  //用户任务结构体
{
  void (*task_func)(u32 dT_us);
  u32 run_time;  //要执行的时间ms
  u8 start_flag;
  u8 end_flag;
} user_task_t;

extern _user_cntrl_word user_cntrl_word;
//==函数声明

// static

// public

void AnoUserCtrl_GetOneByte(uint8_t data);
void AnoUserCtrl_Process(void);
#endif
