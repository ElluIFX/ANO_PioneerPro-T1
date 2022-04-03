#include "Ano_FcData.h"
#include "Ano_Parameter.h"

_switch_st switchs;
 _save_st save;
_flag flag;
_fc_sta_var_st fc_stv;
_sensor_hd_check_st sens_hd_check;



void data_save(void)
{
	para_sta.save_en = !flag.unlock_sta;
	para_sta.save_trig = 1;
}


void Para_Data_Init()
{
//	ANO_Flash_Read((u8 *)(&save),sizeof(save));
//	if(save.first_f != 0x44)
//	{
//		for(u8 i = 0;i<3;i++)
//		{
//			save.gyro_offset[i] = 0;
//		}
//		
//		
//		save.surface_vec[X] = save.surface_vec[Y] = 0;
//		save.surface_vec[Z] = 1;
//	}
	Ano_Parame_Read();
}
