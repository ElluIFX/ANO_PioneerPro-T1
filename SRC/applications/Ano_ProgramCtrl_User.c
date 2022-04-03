#include "Ano_ProgramCtrl_User.h"

#include <stdio.h>

#include "Ano_DT.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlyCtrl.h"
#include "Ano_Math.h"
#include "Drv_Led.h"
#include "Drv_usart.h"

#define USER_TASK_NUM 1  //�û�������

void UserCtrlReset(void);

_pc_user_st pc_user;

_user_cntrl_word user_cntrl_word;  //�û�������

/*-----------------------------------------------------------------------------*/
//�û�����

void empty_task(u32 dT_us) {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  if (user_cntrl_word.mode == 2) {
    user_cntrl_word.land_en = 1;
  }
}

void user_task_point_fix(u32 dT_us) {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  if (user_cntrl_word.mode == 1) {
    user_cntrl_word.break_out = 1;
  }
}

void horiz_stabelize(u32 dT_us) {}

void move_on(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(10, 0); }
void move_back(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(-10, 0); }
void move_left(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(0, 10); }
void move_right(u32 dT_us) { Program_Ctrl_User_Set_HXYcmps(0, -10); }
void turn_left(u32 dT_us) { Program_Ctrl_User_Set_YAWdps(-15 / 2.0f); }
void turn_right(u32 dT_us) { Program_Ctrl_User_Set_YAWdps(45 / 2.0f); }

void user_task_2(u32 dT_us) {}

void user_task_to_barcode(u32 dT_us) {}

void user_task_to_pole(u32 dT_us) {}

void user_task_take_photo(u32 dT_us) {}

void user_task_5(u32 dT_us) {
  if (user_cntrl_word.mode == 3) {
    Program_Ctrl_User_Set_HXYcmps(-10, 0);
  } else {
    user_cntrl_word.break_out = 1;
  }
}

/*------------------------------------------------------------------------------*/
static user_task_t
    user_task[USER_TASK_NUM] =  //�û������б�һ����ɺ��Զ����У�
    {
        {empty_task, 8000, 0, 0},  //
};

//˳��ִ���û�����
static u8 task_index = 0;     //���ڼ�¼����ִ�е�����
static u32 running_time = 0;  //����������ʱ��

void User_Ctrl(u32 dT_ms) {
  if (user_cntrl_word.takeoff_en == 1 && switchs.of_flow_on) {
    //		printf("take off\r\n");
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
  }
  // else if(flag.flying==1&&user_cntrl_word.user_task_running)
  else if (user_cntrl_word.user_task_running) {
    if (task_index < USER_TASK_NUM) {
      if (user_task[task_index].start_flag == 1)  //ִ�е�task_index������
      {
        user_task[task_index].task_func(dT_ms);
        running_time += dT_ms;  //��¼�ۼ�ʱ��
      }

      if (user_task[task_index].end_flag == 1)  //׼����ʼ��һ������
      {
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
        //							 printf("user
        // task<%d> start!\r\n",task_index);
      } else if (running_time >=
                 user_task[task_index].run_time)  //�жϵ�ǰ�����Ƿ����
      {
        user_task[task_index].start_flag = 0;
        user_task[task_index].end_flag = 1;
      }
    } else {
      user_cntrl_word.land_en = 1;
      user_cntrl_word.user_task_running = 0;
      UserCtrlReset();
      FlyCtrlReset();
      task_index = 0;
      running_time = 0;
      //			printf("UserCtrlReset\r\n");
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

void UserCtrlReset() {
  Program_Ctrl_User_Set_HXYcmps(0, 0);
  Program_Ctrl_User_Set_YAWdps(0);
  Program_Ctrl_User_Set_Zcmps(0);
}
