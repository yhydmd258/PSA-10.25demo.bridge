#ifndef _MAX_SERIALIZER_DATA_H_
#define _MAX_SERIALIZER_DATA_H_
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
#ifdef MAX_SERIALIZER_MODULE

/**********************************************************************************************
* Global variables
**********************************************************************************************/

/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define MAX_SER_I2C    I2C1 
#define MAX_COMMUNICATION_TYPE_I2C  0x00

#define MAX_SER_I2C_SET_BYTE_NUMBER   0x01                
typedef struct
{
    UINT8 device_address;
    UINT8 register_address;
//    UINT8 byte_number;
    UINT8 byte[MAX_SER_I2C_SET_BYTE_NUMBER];
}MAX_SERIALIZE_I2C_SET;

typedef struct
{
    UINT8 sync;
    UINT8 dev_addr_rw;
    UINT8 reg_addr;
    UINT8 byte_number;
}MAX_SERIALIZE_UART_SET;

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



/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
extern void Max_Ser_Data_Init(void);
extern void Max_Ser_Receive_Data_Send(void);
#if MAX_COMMUNICATION_TYPE_I2C
extern UINT8 Max_Ser_Data_I2C_Write(UINT8 *send_buf, UINT8 buf_size);
extern UINT8 Max_Ser_Data_I2C_Read(UINT8 dev_add, UINT8 reg_add, UINT8 *read_data);
#else
extern void Max_Ser_Data_UART_Write(MAX_SERIALIZE_UART_SET head, UINT8 *data_buf, UINT8 data_size);
extern void Max_Ser_Data_UART_Read(MAX_SERIALIZE_UART_SET head);
#endif
extern UINT8 Max_Ser_EDID_Cfg_Init(void);

#endif
#endif
