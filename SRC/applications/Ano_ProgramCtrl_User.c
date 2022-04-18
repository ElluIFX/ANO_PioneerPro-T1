#include "Ano_ProgramCtrl_User.h"

#include <stdio.h>

#include "Ano_DT.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlyCtrl.h"
#include "Ano_Math.h"
#include "Drv_Led.h"
#include "Drv_usart.h"

// #define USER_TASK_NUM 1  //�û�������

void UserCtrlReset(void);

_pc_user_st pc_user;

_user_cntrl_word user_cntrl_word;  //�û�������

/*-----------------------------------------------------------------------------*/
//�û�����

void user_task_point_fix(u32 dT_us) {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  Program_Ctrl_User_Set_Zcmps(0.0f);
}

void user_task_remote_ctrl(u32 dT_us) {}

void move_on(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(10, 0); }
void move_back(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(-10, 0); }
void move_left(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(0, 10); }
void move_right(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(0, -10); }
void turn_left(u32 dT_us) { Program_Ctrl_User_Set_YAWdps(-15 / 2.0f); }
void turn_right(u32 dT_us) { Program_Ctrl_User_Set_YAWdps(45 / 2.0f); }

/*------------------------------------------------------------------------------*/
static user_task_t user_task[] =  //�û������б�һ����ɺ��Զ����У�
    {
        {user_task_point_fix, 5000, 0, 0},
        {user_task_remote_ctrl, 1800000, 0, 0},
};
const u8 USER_TASK_NUM = sizeof(user_task) / sizeof(user_task_t);
//˳��ִ���û�����
static u8 task_index = 0;     //���ڼ�¼����ִ�е�����
static u32 running_time = 0;  //����������ʱ��

void User_Ctrl(u32 dT_ms) {
  if (user_cntrl_word.takeoff_en == 1 && switchs.of_flow_on) {
    FlyCtrlReset();
    one_key_take_off();
    user_cntrl_word.takeoff_en = 0;
    user_cntrl_word.user_task_running = 1;  //������ֹ���ֶ�ģʽ�½���usertask
    user_cntrl_word.break_out = 0;  //Ϊ�����������׼��
    task_index = 0;
    running_time = 0;  //����������ʱ��

    for (int i = 0; i < USER_TASK_NUM; i++) {
      user_task[i].end_flag = 0;  //����������������־λ
    }
  } else if (user_cntrl_word.land_en) {
    //			printf("land\r\n");
    UserCtrlReset();
    one_key_land();
    user_cntrl_word.land_en = 0;
    user_cntrl_word.user_task_running = 0;  //��ֹ����
  } else if (flag.auto_take_off_land == AUTO_TAKE_OFF_FINISH &&
             user_cntrl_word.user_task_running) {
    // else if (user_cntrl_word.user_task_running) {
    if (task_index < USER_TASK_NUM) {
      if (user_task[task_index].start_flag == 1)  //ִ�е�task_index������
      {
        user_task[task_index].task_func(dT_ms);
        running_time += dT_ms;  //��¼�ۼ�ʱ��
      }

      if (user_task[task_index].end_flag == 1)  //׼����ʼ��һ������
      {
        DTprintf(" > Task %d END", task_index);
        task_index++;
        running_time = 0;
        FlyCtrlReset();
        UserCtrlReset();
      } else if (user_cntrl_word.break_out == 1)  //�����ǿ�ƽ���
      {
        task_index++;
        running_time = 0;
        user_cntrl_word.break_out = 0;
        user_cntrl_word.stop = 0;
        FlyCtrlReset();
        UserCtrlReset();
      }

      if (running_time == 0)  //������task_index������
      {
        user_task[task_index].start_flag = 1;
      } else if (running_time >=
                 user_task[task_index].run_time)  //�жϵ�ǰ�����Ƿ����
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

//=====1������ˮƽ����ϵ�̿��ٶȹ��ܽӿں���=====
/**********************************************************************************************************
 *�� �� ��: Program_Ctrl_User_Set_HXYcmps
 *����˵��: �̿ع��ܣ�����ˮƽ����ϵ���ٶ��趨��ʵʱ���ƣ�
 *��    ��:
 *X�ٶȣ�����ÿ�룬��Ϊǰ������Ϊ���ˣ�Y�ٶȣ�����ÿ�룬��Ϊ���ƣ���Ϊ���ƣ� ��
 *�� ֵ: ��
 **********************************************************************************************************/
void Program_Ctrl_User_Set_HXYcmps(float hx_vel_cmps, float hy_vel_cmps) {
  //
  pc_user.vel_cmps_set_h[0] = hx_vel_cmps;
  pc_user.vel_cmps_set_h[1] = hy_vel_cmps;
  //����XY�ٶ�ģ��
  length_limit(&pc_user.vel_cmps_set_h[0], &pc_user.vel_cmps_set_h[1],
               MAX_PC_XYVEL_CMPS, pc_user.vel_cmps_set_h);
}

//=====2����ͷģʽ�ο�����ϵ�̿ع��ٶ��ܽӿں��������ޣ��Ժ󿪷������߲ο���λ���̿ع��ܣ�=====
//
//
//

//=====3��ͨ�ó̿��ٶȹ��ܽӿں���=====
/**********************************************************************************************************
 *�� �� ��: Program_Ctrl_User_Set_WHZcmps
 *����˵��: �̿ع��ܣ������½��ٶ��趨��ʵʱ���ƣ�
 *��    ��: �ٶȣ�����ÿ�룬��Ϊ��������Ϊ�½���
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Program_Ctrl_User_Set_Zcmps(float z_vel_cmps) {
  //
  pc_user.vel_cmps_set_z = z_vel_cmps;
  //�޷�
  pc_user.vel_cmps_set_z =
      LIMIT(pc_user.vel_cmps_set_z, -MAX_PC_ZVEL_CMPS, MAX_PC_ZVEL_CMPS);
}
/**********************************************************************************************************
 *�� �� ��: Program_Ctrl_User_Set_YAWdps
 *����˵��: �̿ع��ܣ������ٶ��趨��ʵʱ���ƣ�
 *��    ��: �ٶȣ���ÿ�룬��Ϊ��ת����Ϊ��ת��
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Program_Ctrl_User_Set_YAWdps(float yaw_pal_dps) {
  //
  pc_user.pal_dps_set = yaw_pal_dps;
  //�޷�
  pc_user.pal_dps_set =
      LIMIT(pc_user.pal_dps_set, -MAX_PC_PAL_DPS, MAX_PC_PAL_DPS);
}

/**********************************************************************************************************
 *�� �� ��: Program_Ctrl_User_Set_ZHeight
 *����˵��: �̿ع��ܣ����߸߶��趨
 *��    ��: �߶ȣ����ף�
 *�� �� ֵ: ��
 **********************************************************************************************************/
void Program_Ctrl_User_Set_ZHeight(float z_height) {
  //
  pc_user.height_set = z_height;
  //�޷�
  pc_user.height_set = LIMIT(pc_user.height_set, 10, MAX_PC_HEIGHT);
  pc_user.engage_height_set = 1;
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
 * @brief �ⲿMCU�û�����������պ���,�ڴ����ж��е���
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
  } else if (state == 2)  //������
  {
    state = 3;
    _user_data_temp[2] = data;
  } else if (state == 3)  //����
  {
    state = 4;
    _user_data_temp[3] = data;
    _data_len = data;  //���ݳ���
    _user_data_cnt = 0;
    // if (_data_len == 1) state = 5;
  } else if (state == 4 && _data_len > 0) {
    _data_len--;
    _user_data_temp[4 + _user_data_cnt++] = data;  //����
    if (_data_len == 0) state = 5;
  } else if (state == 5) {
    state = 0;
    _user_data_temp[4 + _user_data_cnt] = data;  // check sum
    _user_data_temp[5 + _user_data_cnt] = 0;
    user_ctrl_data_ok = 1;
  } else
    state = 0;
}

extern u8 send_user_data_flag;

void AnoUserCtrl_Process(void) {
  static u8 option;
  static u8 sub_option;
  static u8 recv_check;
  static u8 calc_check;
  static u8 len;
  static u8 connected = 0;
  static uint8_t* p_data = (uint8_t*)(_user_data_temp + 4);
  static float val1, val2, val3;
  static int16_t temp_s16;
  if (user_ctrl_data_ok) {
    user_ctrl_data_ok = 0;
    option = _user_data_temp[2];
    len = _user_data_temp[3];
    recv_check = _user_data_temp[4 + len];
    calc_check = 0;
    for (u8 i = 0; i < len + 4; i++) {
      calc_check += _user_data_temp[i];
    }
    // DTprintf("R: option:%d,len:%d,unicode:%s", option, len, p_data);
    if (calc_check != recv_check) {
      DTprintf("R: checksum error");
      return;
    }
    switch (option) {
      case 0x00:  // ����
        if (p_data[0] == 0x01) {
          connected = 1;
          send_user_data_flag = 1;
          DTprintf("Ctrl Connected");
          break;
        }
      case 0x01:  // ���̿���
        if (p_data[0] == 0x10) {
          FlyCtrlDataAnl(p_data);
        }
        break;
      case 0x02:  // ʵʱ����
        sub_option = p_data[0];
        temp_s16 = p_data[1] << 8 | p_data[2];
        val1 = temp_s16 / 100.0f;
        temp_s16 = p_data[3] << 8 | p_data[4];
        val2 = temp_s16 / 100.0f;
        // DTprintf("R: sub_option:%d,val1:%f,val2:%f", sub_option, val1, val2);
        switch (sub_option) {
          case 0x01:
            Program_Ctrl_User_Set_HXYcmps(val1, val2);
            break;
          case 0x02:
            Program_Ctrl_User_Set_Zcmps(val1);
            break;
          case 0x03:
            Program_Ctrl_User_Set_YAWdps(val1);
            break;
          case 0x04:
            Program_Ctrl_User_Set_ZHeight(val1);
            break;
          case 0x05:
            Program_Ctrl_User_Set_HXYcmps(0, 0);
            Program_Ctrl_User_Set_Zcmps(0);
            Program_Ctrl_User_Set_YAWdps(0);
            break;
          default:
            break;
        }
        break;
      default:
        break;
    }
  }
}
