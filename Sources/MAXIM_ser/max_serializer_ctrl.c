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
#include "max_serializer_if.h"
#include "max_serializer_ctrl.h"
#include "max_serializer_data.h"
#include "../../Headers/GPIO.h"
#include "../command.h"

/**********************************************************************************************
* Global variables
**********************************************************************************************/

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
static void Max_Ser_Data_Cfg_Init_Before_Edid(void);
static void Max_Ser_Data_Cfg_Init_After_Edid(void);


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
void Max_Ser_Ctrl_Init(void)
{
    Max_Ser_Data_Init();
    Max_Ser_Data_Cfg_Init_Before_Edid();
    Max_Ser_EDID_Cfg_Init();
    Max_Ser_Data_Cfg_Init_After_Edid();
}
/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
void Max_Ser_Ctrl_APP_Data_Send(UINT8* send_buf, UINT32 buf_size)
{
    Max_Ser_If_Data_APP_Send(send_buf, buf_size);
}
/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
void Max_Ser_Ctrl_Task(void)
{
    Max_Ser_Receive_Data_Send();
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Max_Ser_Data_Cfg_Init_Before_Edid(void)
{
#if MAX_COMMUNICATION_TYPE_I2C
//    UINT8 read=0;
    MAX_SERIALIZE_I2C_SET ser_I2C_set;
    UINT8 result=0;

//    result=Max_Ser_Data_I2C_Read(0x80,0x95,&read);/*0x80,0x95,0x81,0x00*/
//    read=0;
//    result=Max_Ser_Data_I2C_Read(0x90,0x95,&read);/*0x90,0x95,0x91,0xFF*/

    /* Auto software reset whenever SCDT=0 */
    ser_I2C_set.device_address = 0x60;
    ser_I2C_set.register_address = 0x05;
    ser_I2C_set.byte[0] = 0xFF;
    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));

//    ser_I2C_set.device_address = 0x60;
//    ser_I2C_set.register_address = 0x05;
//    ser_I2C_set.byte[0] = 0x90;
//    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));

//    ser_I2C_set.device_address = 0x60;
//    ser_I2C_set.register_address = 0xF9;
//    ser_I2C_set.byte[0] = 0x01;
//    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));

//    ser_I2C_set.device_address = 0x60;
//    ser_I2C_set.register_address = 0xFA;
//    ser_I2C_set.byte[0] = 0x01;
//    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));
    
    ser_I2C_set.device_address = 0x60;
    ser_I2C_set.register_address = 0x91;
    ser_I2C_set.byte[0] = 0x01;
    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));

//    read=0;
//    result=Max_Ser_Data_I2C_Read(0x60,0x03,&read);/*0x60,0x03,0x61,0x91*/

//    read=0;
//    result=Max_Ser_Data_I2C_Read(0x60,0x02,&read);/*0x60,0x02,0x61,0x35*/
    
//    read=0;
//    result=Max_Ser_Data_I2C_Read(0x60,0x04,&read);/*0x60,0x04,0x61,0x00*/
    
//    des_set.device_address = 0x60;
//    des_set.register_address = 0x05;
//    des_set.byte[0] = 0xFF;
//    result=Max_Ser_Data_I2C_Write((UINT8*)&des_set, sizeof(MAX_SERIALIZE_I2C_SET));
    
    ser_I2C_set.device_address = 0x60;
    ser_I2C_set.register_address = 0x09;
    ser_I2C_set.byte[0] = 0x90;
    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));
    
    ser_I2C_set.device_address = 0xF0;
    ser_I2C_set.register_address = 0xE9;
    ser_I2C_set.byte[0] = 0x01;
    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));
    
    ser_I2C_set.device_address = 0xF0;
    ser_I2C_set.register_address = 0xEA;
    ser_I2C_set.byte[0] = 0x01;
    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));

//    des_set.device_address = 0x60;
//    des_set.register_address = 0x91;
//    des_set.byte[0] = 0x01;
//    result=Max_Ser_Data_I2C_Write((UINT8*)&des_set, sizeof(MAX_SERIALIZE_I2C_SET));
    
#else
    MAX_SERIALIZE_UART_SET  ser_uart_set;
    UINT8   send_data[10]={0};
    INT32  delay = 0x00;

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0x60;
    ser_uart_set.reg_addr = 0x05;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0xFF;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0x60;
    ser_uart_set.reg_addr = 0x91;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0x01;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);
    
    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0x60;
    ser_uart_set.reg_addr = 0x09;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0x90;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0xF0;
    ser_uart_set.reg_addr = 0xE9;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0x01;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0xF0;
    ser_uart_set.reg_addr = 0xEA;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0x01;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);

#endif
}
/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Max_Ser_Data_Cfg_Init_After_Edid(void)
{
#if MAX_COMMUNICATION_TYPE_I2C
//    UINT8 read=0;
    MAX_SERIALIZE_I2C_SET ser_I2C_set;
    UINT8 result=0;
//    read=0;
//    result=Max_Ser_Data_I2C_Read(0x90,0x04,&read);/*0x90,0x04,0x91,0xFF*/

//    read=0;
//    result=Max_Ser_Data_I2C_Read(0x60,0x05,&read);/*0x60,0x05,0x61,0x00*/
    
    ser_I2C_set.device_address = 0x60;
    ser_I2C_set.register_address = 0x05;
    ser_I2C_set.byte[0] = 0x00;
    result=Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));

//    read=0;
//    result=Max_Ser_Data_I2C_Read(0xA0,0x49,&read);/*0xA0,0x49,0xA1,0x00*/

//    des_set.device_address = 0xA0;
//    des_set.register_address = 0x49;
//    des_set.byte[0] = 0x00;
//    result=Max_Ser_Data_I2C_Write((UINT8*)&des_set, sizeof(MAX_SERIALIZE_I2C_SET));
    
//    read=0;
//    result=Max_Ser_Data_I2C_Read(0xA0,0x49,&read);/*0xA0,0x49,0xA1,0x00*/
/* initialize des_set, set I2C-to-UART */    
    ser_I2C_set.device_address = CMD_DESER_DEV_ADDR;
    ser_I2C_set.register_address = 0x04;
    ser_I2C_set.byte[0] = 0x07;
    Max_Ser_Data_I2C_Write((UINT8*)&ser_I2C_set, sizeof(MAX_SERIALIZE_I2C_SET));
#else
    MAX_SERIALIZE_UART_SET  ser_uart_set;
    UINT8   send_data[10]={0};
    INT32  delay = 0x00;

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0x60;
    ser_uart_set.reg_addr = 0x05;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0x00;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);
    
    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = CMD_DESER_DEV_ADDR;
    ser_uart_set.reg_addr = 0x04;
    ser_uart_set.byte_number = 0x01;
    send_data[0] = 0x07;
    Max_Ser_Data_UART_Write(ser_uart_set, send_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = 0x60;
    ser_uart_set.reg_addr = 0x05;
    ser_uart_set.byte_number = 0x01;
    Max_Ser_Data_UART_Read(ser_uart_set);
/*new*/	
//	CONFIG_PIN_AS_GPIO(PTF,PTF7,OUTPUT); /* Configure MS as an output */
    OUTPUT_SET(PTF,PTF7);       		/* bypass mode */
	OUTPUT_SET(PTF,PTF6);

#endif
}
#endif
