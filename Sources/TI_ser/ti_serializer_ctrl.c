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
#include <string.h>
#include "ti_serializer_if.h"
#include "ti_serializer_ctrl.h"
#include "ti_serializer_data.h"
#include "atmel_1189.h"

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
static UINT8 Chip_Detected_Flag = false;
TOUCH_COORDINATE_BUF Touch_Coordinate_Buf;

/**********************************************************************************************
* Local functions
**********************************************************************************************/
static void Ti_Ser_Reverse_Comm_Enable(void);
static void Ti_Ser_Ctrl_CTP_Init(void);
static void Ti_Ser_Ctrl_CTP_Touch_Init(void);
static void Ti_Ser_EDID_Cfg_Init_Before_EDID(void);

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
void Ti_Ser_Ctrl_Init(void)
{
    UINT32 i=0x00;
    Ti_Ser_Data_Init();
    i=0x1F40;/* delay 1ms */
    while(i--);
    Ti_Ser_Reverse_Comm_Enable();
    Ti_Ser_EDID_Cfg_Init_Before_EDID();
    Ti_Ser_EDID_Cfg_Init();
//    Ti_Ser_Ctrl_CTP_Init();
//    Ti_Ser_Ctrl_CTP_Touch_Init();
}

/***********************************************************************************************
*
* @brief    CTP_If_Init() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Ti_Ser_Ctrl_CTP_Task(void)
{
#if 0
    if(Chip_Detected_Flag == true)
    {  
        if(Ti_Ser_Data_Interrupt_Read() == CHANGELINE_NEGATED)
        {
            get_touch_message_for_PSA(&Touch_Coordinate_Buf);
            Ti_Ser_Data_Cmd_make(&Touch_Coordinate_Buf);
        }
    }
#endif
#if 0
    UINT8 send_buf[6]={0x2C,0x10,0x20,0x30,0x40,0x00};
    UINT32 delay_time=800000;

    while(delay_time--);// delay more than 100ms;
    
    Ti_Ser_Data_I2C_Write(send_buf, 6);
#endif
}

/***********************************************************************************************
*
* @brief    CTP_If_Init() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Ti_Ser_Ctrl_Data_Send(UINT8* send_buf, UINT32 buf_size)
{
    Ti_Ser_If_Data_Send(send_buf, buf_size);
}
/***********************************************************************************************
*
* @brief    CTP_If_Init() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Ctrl_CTP_Init(void)
{
    Touch_Coordinate_Buf.write_pos=0;
    Touch_Coordinate_Buf.read_pos=COORDINATE_BUF_NUMBER-1;
    memset(Touch_Coordinate_Buf.buf, 0x00, sizeof(Touch_Coordinate_Buf.buf));
    Touch_Coordinate_Buf.action=0;
}

/***********************************************************************************************
*
* @brief    CTP_If_Init() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Ctrl_CTP_Touch_Init(void)
{
    UINT32 i;
    
    i=0x000B0000;/*90.2ms*/
    while(i--);//  delay 
    
    Chip_Detected_Flag = init_touch_app();  // find and initialise QT device 
    if(Chip_Detected_Flag == true)
    {
        while(Ti_Ser_Data_Interrupt_Read() == CHANGELINE_NEGATED)
        {
            get_message();
        }
    }
}
/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_EDID_Cfg_Init_Before_EDID(void)
{
    UINT8 receive_buf;
    TI_SERIALIZE_I2C_SET    send_data={0};

    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x4F,&receive_buf); /* receive_buf: 0x00*/
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x4F;
    send_data.byte[0] = 0x00;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
    
    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x54,&receive_buf); /* receive_buf: 0x28*/
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x54;
    send_data.byte[0] = 0x00;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x54,&receive_buf); /* receive_buf: 0x28*/

    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x48,&receive_buf); /* receive_buf: 0x00*/
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x48;
    send_data.byte[0] = 0x0D;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x51,&receive_buf); /* receive_buf: 0x20*/
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x51;
    send_data.byte[0] = 0xA0;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    /* set the offset postion of EDID data */
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x49;
    send_data.byte[0] = 0x00;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

//    Ti_Ser_Data_I2C_Read(TI_DESER_I2C_DEV_ADDRESS,0x49,&receive_buf); /* receive_buf: 0x03*/

    /* add for deserializer(948), serializer(929)*/
    send_data.device_address = TI_DESER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x49;
//    send_data.byte[0] = 0x63;
    send_data.byte[0] = 0x60;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

//    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x51,&receive_buf); /* receive_buf: 0x20*/

//    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x48,&receive_buf); /* receive_buf: 0x20*/

//    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x5B,&receive_buf);
//    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
//    send_data.register_address = 0x48;
//    send_data.byte[0] = 0x08;
//    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
    
//    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x54,&receive_buf);
//    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
//    send_data.register_address = 0x54;
//    send_data.byte[0] = 0x00;
//    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Reverse_Comm_Enable(void)
{
    TI_SERIALIZE_I2C_SET    send_data={0};
#if 0
    /* for GM 8inch */
    /* send data to serializer */
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x03;
    send_data.byte[0] = 0xFA;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
    
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x07;
    send_data.byte[0] = 0x96;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x08;
    send_data.byte[0] = 0x96;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    /* send data to deserializer */
    send_data.device_address = TI_DESER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x03;
    send_data.byte[0] = 0xF8;
//    send_data.byte[0] = 0xB8;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    send_data.device_address = TI_DESER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x05;
    send_data.byte[0] = 0x9E;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    /* enable intterupt pin, */
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0xC6;
    send_data.byte[0] = 0x21;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
#else
    /* for GM 26.3inch*/
    /* send data to serializer */
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x03;
    send_data.byte[0] = 0xFA;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
    
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x07;
    send_data.byte[0] = TI_DESER_I2C_APP_DEV_ADDRESS;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x08;
    send_data.byte[0] = TI_DESER_I2C_APP_DEV_ADDRESS;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    /* send data to deserializer */
    send_data.device_address = TI_DESER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x03;
    send_data.byte[0] = 0xF8;
//    send_data.byte[0] = 0xB8;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    send_data.device_address = TI_DESER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x05;
    send_data.byte[0] = 0x9E;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    /* enable intterupt pin, */
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0xC6;
    send_data.byte[0] = 0x21;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
#endif
}
#endif
