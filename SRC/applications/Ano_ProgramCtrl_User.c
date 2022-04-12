#include "Ano_ProgramCtrl_User.h"

#include <stdio.h>

#include "Ano_DT.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlyCtrl.h"
#include "Ano_Math.h"
#include "Drv_Led.h"
#include "Drv_usart.h"

// #define USER_TASK_NUM 1  //用户任务数

void UserCtrlReset(void);

_pc_user_st pc_user;

_user_cntrl_word user_cntrl_word;  //用户控制字

/*-----------------------------------------------------------------------------*/
//用户任务

void user_task_point_fix(u32 dT_us) {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  Program_Ctrl_User_Set_Zcmps(0.0f);
}

void user_task_remote_ctrl(u32 dT_us) {
  if (user_cntrl_word.engage_hxy) {
    Program_Ctrl_User_Set_HXYcmps(user_cntrl_word.hx, user_cntrl_word.hy);
  }
  if (user_cntrl_word.engage_yaw) {
    Program_Ctrl_User_Set_YAWdps(user_cntrl_word.yaw);
  }
  if (user_cntrl_word.engage_z) {
    Program_Ctrl_User_Set_Zcmps(user_cntrl_word.z);
  }
}

void move_on(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(10, 0); }
void move_back(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(-10, 0); }
void move_left(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(0, 10); }
void move_right(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(0, -10); }
void turn_left(u32 dT_us) { Program_Ctrl_User_Set_YAWdps(-15 / 2.0f); }
void turn_right(u32 dT_us) { Program_Ctrl_User_Set_YAWdps(45 / 2.0f); }

/*------------------------------------------------------------------------------*/
static user_task_t user_task[] =  //用户任务列表（一键起飞后自动进行）
    {
        {user_task_point_fix, 5000, 0, 0},
        {user_task_remote_ctrl, 1800000, 0, 0},
};
const u8 USER_TASK_NUM = sizeof(user_task) / sizeof(user_task_t);
//顺序执行用户任务
static u8 task_index = 0;     //用于记录正在执行的任务
static u32 running_time = 0;  //任务已运行时间

void User_Ctrl(u32 dT_ms) {
  if (user_cntrl_word.takeoff_en == 1 && switchs.of_flow_on) {
    FlyCtrlReset();
    one_key_take_off();
    user_cntrl_word.takeoff_en = 0;
    user_cntrl_word.user_task_running = 1;  //用来防止纯手动模式下进入usertask
    user_cntrl_word.break_out = 0;  //为任务的跳出做准备
    task_index = 0;
    running_time = 0;  //任务已运行时间

    user_cntrl_word.engage_hxy = 0;
    user_cntrl_word.engage_yaw = 0;
    user_cntrl_word.engage_z = 0;
    user_cntrl_word.hx = 0;
    user_cntrl_word.hy = 0;
    user_cntrl_word.yaw = 0;
    user_cntrl_word.z = 0;

    for (int i = 0; i < USER_TASK_NUM; i++) {
      user_task[i].end_flag = 0;  //清空所有任务结束标志位
    }
  } else if (user_cntrl_word.land_en) {
    //			printf("land\r\n");
    UserCtrlReset();
    one_key_land();
    user_cntrl_word.land_en = 0;
    user_cntrl_word.user_task_running = 0;  //终止任务
  } else if (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH &&
             user_cntrl_word.user_task_running) {
    // else if (user_cntrl_word.user_task_running) {
    if (task_index < USER_TASK_NUM) {
      if (user_task[task_index].start_flag == 1)  //执行第task_index个任务
      {
        user_task[task_index].task_func(dT_ms);
        running_time += dT_ms;  //记录累计时间
      }

      if (user_task[task_index].end_flag == 1)  //准备开始下一个任务
      {
        DTprintf(" > Task %d END", task_index);
        task_index++;
        running_time = 0;
        FlyCtrlReset();
        UserCtrlReset();
      } else if (user_cntrl_word.break_out == 1)  //任务的强制结束
      {
        task_index++;
        running_time = 0;
        user_cntrl_word.break_out = 0;
        user_cntrl_word.stop = 0;
        FlyCtrlReset();
        UserCtrlReset();
      }

      if (running_time == 0)  //启动第task_index个任务
      {
        user_task[task_index].start_flag = 1;
      } else if (running_time >=
                 user_task[task_index].run_time)  //判断当前任务是否完成
      {
        user_task[task_index].start_flag = 0;
        user_task[task_index].end_flag = 1;
      }
    } else {
      ANO_DT_SendString(" > All task finished!");
      user_cntrl_word.land_en = 1;
      user_cntrl_word.user_task_running = 0;
      UserCtrlReset();
      FlyCtrlReset();
      task_index = 0;
      running_time = 0;
    }
  }
}

//=====1、航向水平坐标系程控速度功能接口函数=====
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_HXYcmps
 *功能说明: 程控功能，航向水平坐标系下速度设定（实时控制）
 *形    参:
 *X速度（厘米每秒，正为前进，负为后退，Y速度（厘米每秒，正为左移，负为右移） 返
 *回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps) {
  //
  pc_user.vel_cmps_set_h[0] = hx_vel_cmps;
  pc_user.vel_cmps_set_h[1] = hy_vel_cmps;
  //限制XY速度模长
  length_limit(&pc_user.vel_cmps_set_h[0], &pc_user.vel_cmps_set_h[1],
               MAX_PC_XYVEL_CMPS, pc_user.vel_cmps_set_h);
}

//=====2、无头模式参考坐标系程控功速度能接口函数（暂无，稍后开发，或者参考上位机程控功能）=====
//
//
//

//=====3、通用程控速度功能接口函数=====
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_WHZcmps
 *功能说明: 程控功能，上升下降速度设定（实时控制）
 *形    参: 速度（厘米每秒，正为上升，负为下降）
 *返 回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps) {
  //
  pc_user.vel_cmps_set_z = z_vel_cmps;
  //限幅
  pc_user.vel_cmps_set_z =
      LIMIT(pc_user.vel_cmps_set_z, -MAX_PC_ZVEL_CMPS, MAX_PC_ZVEL_CMPS);
}
/**********************************************************************************************************
 *函 数 名: Program_Ctrl_User_Set_YAWdps
 *功能说明: 程控功能，航向速度设定（实时控制）
 *形    参: 速度（度每秒，正为右转，负为左转）
 *返 回 值: 无
 **********************************************************************************************************/
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps) {
  //
  pc_user.pal_dps_set = yaw_pal_dps;
  //限幅
  pc_user.pal_dps_set =
      LIMIT(pc_user.pal_dps_set, -MAX_PC_PAL_DPS, MAX_PC_PAL_DPS);
}

void UserCtrlReset() {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  Program_Ctrl_User_Set_Zcmps(0);
}

static uint8_t _user_data_temp[50];
static u8 _user_data_cnt = 0;
static u8 user_ctrl_data_ok = 0;

/**
 * @brief 外部MCU用户控制命令接收函数,在串口中断中调用
 * @param  data
 */
void AnoUserCtrl_GetOneByte(uint8_t data) {
  static u8 _data_len = 0;
  static u8 state = 0;

  if (state == 0 && data == 0xAA) {
    state = 1;
    _user_data_temp[0] = data;
  } else if (state == 1 && data == 0x22) {
    state = 2;
    _user_data_temp[1] = data;
    user_ctrl_data_ok = 0;
  } else if (state == 2)  //功能字
  {
    state = 3;
    _user_data_temp[2] = data;
  } else if (state == 3)  //长度
  {
    state = 4;
    _user_data_temp[3] = data;
    _data_len = data;
    _user_data_cnt = 0;
    if (_data_len == 1) state = 5;
  } else if (state == 4 && _data_len > 0) {
    _data_len--;
    _user_data_temp[4 + _user_data_cnt++] = data;
    if (_data_len == 1) state = 5;
  } else if (state == 5) {
    state = 0;
    _user_data_temp[4 + _user_data_cnt] = data;
    _user_data_temp[5 + _user_data_cnt] = 0;
    user_ctrl_data_ok = 1;
  } else
    state = 0;
}

extern u8 send_user_data_flag;

void AnoUserCtrl_Process(void) {
  static u8 option;
  static u8 len;
  static u8 connected = 0;
  static uint8_t* p_data = (uint8_t*)(_user_data_temp + 4);
  if (user_ctrl_data_ok) {
    user_ctrl_data_ok = 0;
    option = _user_data_temp[2];
    len = _user_data_temp[3];
    DTprintf("R: option:%d,len:%d,unicode:%s", option, len, p_data);
    switch (option) {
      case 0x00:  // 握手
        if (p_data[0] == 0x01) {
          connected = 1;
          send_user_data_flag = 1;
          DTprintf("Ctrl Connected");
          break;
        }
      case 0x01:
      case 0x02:
      default:
        break;
    }
  }
}
