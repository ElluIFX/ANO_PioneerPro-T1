#include "Ano_FlightCtrl.h"

#include "Ano_AltCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_FlyCtrl.h"
#include "Ano_Imu.h"
#include "Ano_LocCtrl.h"
#include "Ano_MagProcess.h"
#include "Ano_MotionCal.h"
#include "Ano_MotorCtrl.h"
#include "Ano_OF.h"
#include "Ano_RC.h"
#include "Drv_icm20602.h"
#include "Drv_led.h"
#include "Drv_spl06.h"
#include "Drv_vl53l0x.h"

/*用户添加*/
#include "Ano_ProgramCtrl_User.h"
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////

/*PID参数初始化*/
void All_PID_Init(void) {
  /*姿态控制，角速度PID初始化*/
  Att_1level_PID_Init();

  /*姿态控制，角度PID初始化*/
  Att_2level_PID_Init();

  /*高度控制，高度速度PID初始化*/
  Alt_1level_PID_Init();

  /*高度控制，高度PID初始化*/
  Alt_2level_PID_Init();

  /*位置速度控制PID初始化*/
  Loc_1level_PID_Init();
}

/*控制参数改变任务*/
void ctrl_parameter_change_task() {
  if (0) {
    Set_Att_2level_Ki(0);

  } else {
    if (flag.auto_take_off_land == AUTO_TAKE_OFF) {
      Set_Att_1level_Ki(2);
    } else {
      Set_Att_1level_Ki(1);
    }

    Set_Att_2level_Ki(1);
  }
}

/*一键翻滚（暂无）*/
void one_key_roll() {
  if (flag.flying && flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH) {
    if (rolling_flag.roll_mode == 0) {
      rolling_flag.roll_mode = 1;
    }
  }
}

static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms) {
  if (one_key_taof_start != 0) {
    one_key_taof_start += dt_ms;

    if (one_key_taof_start > 1400 && flag.motor_preparation == 1) {
      one_key_taof_start = 0;
      if (flag.auto_take_off_land == AUTO_TAKE_OFF_NULL) {
        flag.auto_take_off_land = AUTO_TAKE_OFF;
        flag.taking_off = 1;
      }
    }
  }
  // reset
  if (flag.unlock_sta == 0) {
    one_key_taof_start = 0;
  }
}
/*一键起飞*/
void one_key_take_off() {
  if (flag.unlock_err == 0) {
    if (flag.auto_take_off_land == AUTO_TAKE_OFF_NULL &&
        one_key_taof_start == 0) {
      one_key_taof_start = 1;
      flag.unlock_cmd = 1;
    }
  }
}
/*一键降落*/
void one_key_land() { flag.auto_take_off_land = AUTO_LAND; }

//////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////
_flight_state_st fs;

s16 flying_cnt, landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*降落检测*/

static s16 ld_delay_cnt;
void land_discriminat(s16 dT_ms) {
  //	static s16 acc_delta,acc_old;

  //	acc_delta = imu_data.w_acc[Z]- acc_old;
  //	acc_old = imu_data.w_acc[Z];

  /*油门归一值小于0.1  或者启动自动降落*/
  if ((fs.speed_set_h_norm[Z] < 0.1f) || flag.auto_take_off_land == AUTO_LAND) {
    if (ld_delay_cnt > 0) {
      ld_delay_cnt -= dT_ms;
    }
  } else {
    ld_delay_cnt = 200;
  }

  /*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2
   * 保持200ms才开始检测*/
  if (ld_delay_cnt <= 0 &&
      (flag.thr_low || flag.auto_take_off_land == AUTO_LAND)) {
    /*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1秒，认为着陆，然后上锁*/
    if (mc.ct_val_thr < 250 && flag.unlock_sta == 1 &&
        flag.locking != 2)  // ABS(wz_spe_f1.out <20 ) //还应当
                            // 与上速度条件，速度小于正20厘米每秒。
    {
      if (landing_cnt < 1500) {
        landing_cnt += dT_ms;
      } else {
        flying_cnt = 0;
        flag.taking_off = 0;
        ///////
        landing_cnt = 0;
        flag.unlock_cmd = 0;

        flag.flying = 0;
      }
    } else {
      landing_cnt = 0;
    }

  } else {
    landing_cnt = 0;
  }
}

/*飞行状态任务*/

void Flight_State_Task(u8 dT_ms, s16 *CH_N) {
  s16 thr_deadzone;
  static float max_speed_lim, vel_z_tmp[2];
  /*设置油门摇杆量*/
  thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
  fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR], 0, thr_deadzone) * 0.0023f;
  fs.speed_set_h_norm_lpf[Z] +=
      0.5f * (fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);

  /*推油门起飞*/
  if (flag.unlock_sta) {
    if (fs.speed_set_h_norm[Z] > 0.01f && flag.motor_preparation == 1)  // 0-1
    {
      flag.taking_off = 1;
    }
  }
  //
  fc_stv.vel_limit_z_p = MAX_Z_SPEED_UP;
  fc_stv.vel_limit_z_n = -MAX_Z_SPEED_DW;
  //
  if (flag.taking_off) {
    if (flying_cnt < 1000)  // 800ms
    {
      flying_cnt += dT_ms;
    } else {
      /*起飞后1秒，认为已经在飞行*/
      flag.flying = 1;
    }

    if (fs.speed_set_h_norm[Z] > 0) {
      /*设置上升速度*/
      vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] * MAX_Z_SPEED_UP);
    } else {
      /*设置下降速度*/
      vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] * MAX_Z_SPEED_DW);
    }

    //飞控系统Z速度目标量综合设定
    vel_z_tmp[1] =
        vel_z_tmp[0] + program_ctrl.vel_cmps_h[Z] + pc_user.vel_cmps_set_z;
    //
    vel_z_tmp[1] =
        LIMIT(vel_z_tmp[1], fc_stv.vel_limit_z_n, fc_stv.vel_limit_z_p);
    //
    fs.speed_set_h[Z] +=
        LIMIT((vel_z_tmp[1] - fs.speed_set_h[Z]), -0.8f, 0.8f);  //限制增量幅度
  } else {
    fs.speed_set_h[Z] = 0;
  }
  float speed_set_tmp[2];
  /*速度设定量，正负参考ANO坐标参考方向*/
  fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT], 0, 50) * 0.0022f);
  fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL], 0, 50) * 0.0022f);

  LPF_1_(3.0f, dT_ms * 1e-3f, fs.speed_set_h_norm[X],
         fs.speed_set_h_norm_lpf[X]);
  LPF_1_(3.0f, dT_ms * 1e-3f, fs.speed_set_h_norm[Y],
         fs.speed_set_h_norm_lpf[Y]);

  max_speed_lim = MAX_SPEED;

  if (switchs.of_flow_on) {
    max_speed_lim = 1.5f * wcz_hei_fus.out;
    max_speed_lim = LIMIT(max_speed_lim, 50, 150);
  }

  fc_stv.vel_limit_xy = max_speed_lim;

  //飞控系统XY速度目标量综合设定
  speed_set_tmp[X] = fc_stv.vel_limit_xy * fs.speed_set_h_norm_lpf[X] +
                     program_ctrl.vel_cmps_h[X] + pc_user.vel_cmps_set_h[X];
  speed_set_tmp[Y] = fc_stv.vel_limit_xy * fs.speed_set_h_norm_lpf[Y] +
                     program_ctrl.vel_cmps_h[Y] + pc_user.vel_cmps_set_h[Y];

  length_limit(&speed_set_tmp[X], &speed_set_tmp[Y], fc_stv.vel_limit_xy,
               fs.speed_set_h_cms);

  fs.speed_set_h[X] = fs.speed_set_h_cms[X];
  fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];

  /*调用检测着陆的函数*/
  land_discriminat(dT_ms);

  /*倾斜过大上锁*/
  if (rolling_flag.rolling_step == ROLL_END) {
    if (imu_data.z_vec[Z] <
        0.25f)  // 75度
                // ////////////////////////////////////////***************************
                // 倾斜过大上锁，慎用。
    {
      flag.unlock_cmd = 0;
    }
  }
  //////////////////////////////////////////////////////////
  /*校准中，复位重力方向*/
  if (sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||
      sensor.acc_z_auto_CALIBRATE) {
    imu_state.G_reset = 1;
  }

  /*复位重力方向时，认为传感器失效*/
  if (imu_state.G_reset == 1) {
    flag.sensor_imu_ok = 0;
    LED_STA.rst_imu = 1;
    WCZ_Data_Reset();  //复位高度数据融合
  } else if (imu_state.G_reset == 0) {
    if (flag.sensor_imu_ok == 0) {
      flag.sensor_imu_ok = 1;
      LED_STA.rst_imu = 0;
      ANO_DT_SendString("IMU OK!");
    }
  }

  /*飞行状态复位*/
  if (flag.unlock_sta == 0) {
    flag.flying = 0;
    landing_cnt = 0;
    flag.taking_off = 0;
    flying_cnt = 0;

    flag.rc_loss_back_home = 0;

    //复位融合
    if (flag.taking_off == 0) {
      //			wxyz_fusion_reset();
    }
  }
}

static u8 of_light_ok;
static u16 of_light_delay;
void Swtich_State_Task(u8 dT_ms) {
  switchs.baro_on = 1;

  if (sens_hd_check.of_ok)  //光流模块
  {
    if (OF_QUALITY >
        20)  //|| flag.flying == 0) //光流质量大于20
             ///*或者在飞行之前*/，认为光流可用，判定可用延迟时间为1秒
    {
      if (of_light_delay < 1000) {
        of_light_delay += dT_ms;
      } else {
        of_light_ok = 1;
      }
    } else {
      of_light_delay = 0;
      of_light_ok = 0;
    }

    if (OF_ALT < 500 && flag.flight_mode == LOC_HOLD)  //光流高度500cm内有效
    {
      if (of_light_ok) {
        switchs.of_flow_on = 1;
      } else {
        switchs.of_flow_on = 0;
      }
      switchs.of_tof_on = 1;
    } else {
      switchs.of_tof_on = 0;
      switchs.of_flow_on = 0;
    }
  } else {
    switchs.of_flow_on = switchs.of_tof_on = 0;
  }

  if (sens_hd_check.tof_ok)  //激光模块
  {
    if (tof_height_mm < 1900) {
      switchs.tof_on = 1;
    } else {
      switchs.tof_on = 0;
    }
  } else {
    switchs.tof_on = 0;
  }
}

// static void Speed_Mode_Switch() {
  //	if( ubx_user_data.s_acc_cms > 60)// || ubx_user_data.svs_used < 6)
  //	{
  //		flag.speed_mode = 0;
  //	}
  //	else
  //	{
  //		flag.speed_mode = 1;
  //	}
// }

u8 speed_mode_old = 255;
u8 flight_mode_old = 255;
void Flight_Mode_Set(u8 dT_ms) {
  // Speed_Mode_Switch();

  if (speed_mode_old != flag.speed_mode)  //状态改变
  {
    speed_mode_old = flag.speed_mode;
    // xy_speed_pid_init(flag.speed_mode);////////////
  }

  ///////////////////////////////////////////////////////

  if (CH_N[AUX1] < -100 &&
      CH_N[AUX1] > -200)  //接收机失控值，需要手工设置遥控器
  {
    //遥控设置的接收机输出的失控保护的信号。
    flag.chn_failsafe = 1;
  } else {
    flag.chn_failsafe = 0;
    ////
    if (CH_N[AUX1] < -300) {
      flag.flight_mode = ATT_STAB;
    } else if (CH_N[AUX1] < 200) {
      flag.flight_mode = LOC_HOLD;
    } else {
      flag.flight_mode = RETURN_HOME;
    }
  }

  if (flight_mode_old != flag.flight_mode)  //摇杆对应模式状态改变
  {
    flight_mode_old = flag.flight_mode;

    flag.rc_loss_back_home = 0;
  }
}
