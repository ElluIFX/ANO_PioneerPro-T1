#ifndef __DRV_UP_Flow_H
#define __DRV_UP_Flow_H

//==����
//#include "sysconfig.h"
#include "Ano_FcData.h"

//==����

//==��������
extern uint8_t of_buf_update_cnt;
extern uint8_t OF_DATA[];
//==��������

//static


//public
void Drv_OFInit(void);
void OFGetByte(uint8_t data);

#endif
