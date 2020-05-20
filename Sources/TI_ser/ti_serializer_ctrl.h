#ifndef _TI_SERIALIZER_CTRL_H_
#define _TI_SERIALIZER_CTRL_H_
/******************************************************************************								
*  Name: .c
*  Description: 
*  Project: C_demo
*  Auther:  shanghai tianma
*              R&D center
*              penghu
*  MCU: S9KEAZ128AMLH
*  Comment:
******************************************************************************/		
/**********************************************************************************************
* External objects
**********************************************************************************************/
#include "../../Headers/derivative.h" /* include peripheral declarations */
#ifdef TI_SERIALIZER_MODULE

#define TI_SER_I2C    I2C1 
#define CTP_IC  TI_SER_I2C
#define COORDINATE_BUF_NUMBER 100
/**********************************************************************************************
* Global variables
**********************************************************************************************/
typedef struct
{
    UINT8 write_pos;
    UINT8 read_pos;
    UINT8 buf[COORDINATE_BUF_NUMBER][6];
    UINT8 action;
}TOUCH_COORDINATE_BUF;

/**********************************************************************************************
* Constants and macros
**********************************************************************************************/

/**********************************************************************************************
* Local types
**********************************************************************************************/


/**********************************************************************************************
* Local function prototypes
*********************************************************************************************/

/**********************************************************************************************
* Local variables
**********************************************************************************************/


/**********************************************************************************************
* Local functions
**********************************************************************************************/


/**********************************************************************************************
* Global functions
**********************************************************************************************/
extern void Ti_Ser_Ctrl_Init(void);
extern void Ti_Ser_Ctrl_CTP_Task(void);
extern void Ti_Ser_Ctrl_Data_Send(UINT8* send_buf, UINT32 buf_size);

#endif
#endif