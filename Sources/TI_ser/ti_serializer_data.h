#ifndef _TI_SERIALIZER_DATA_H_
#define _TI_SERIALIZER_DATA_H_
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
#include "ti_serializer_ctrl.h"

/**********************************************************************************************
* Global variables
**********************************************************************************************/

/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define TI_SER_I2C_SET_BYTE_NUMBER   0x01                
#define TI_SER_I2C_DEV_ADDRESS      0x18    /* MODE_SEL0 and MODE_SEL1 is 1.44V */
#define TI_DESER_I2C_DEV_ADDRESS    0x58
#define TI_DESER_I2C_APP_DEV_ADDRESS    0x2C
typedef struct
{
    UINT8 device_address;
    UINT8 register_address;
    UINT8 byte[TI_SER_I2C_SET_BYTE_NUMBER];
}TI_SERIALIZE_I2C_SET;

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
extern void Ti_Ser_Data_Init(void);
extern UINT8 Ti_Ser_Data_I2C_Write(UINT8 *send_buf, UINT8 buf_size);
extern UINT8 Ti_Ser_Data_I2C_Read(UINT8 dev_add, UINT8 reg_add, UINT8 *read_data);
extern void Ti_Ser_EDID_Cfg_Init(void);
extern UINT8 Ti_Ser_Data_Interrupt_Read(void);
extern void Ti_Ser_Data_Cmd_make(TOUCH_COORDINATE_BUF *touch_data);

#endif
#endif
