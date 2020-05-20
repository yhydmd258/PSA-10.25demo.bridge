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
#include <stdio.h>
#include "../../Headers/GPIO.h"
#include "../../Headers/I2C.h"
#include "ti_serializer_data.h"
#include "ti_serializer_ctrl.h"
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
static void Ti_Ser_Data_Power_Init(void);
static void Ti_Ser_Data_PDB_Init(void);
static void Ti_Ser_Data_I2C_Init(void);
static void Ti_Ser_Data_Interrupt_Init(void);

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
void Ti_Ser_Data_Init(void)
{
    UINT32 i=0x50;/* delay 10us */
    /* Ti power on sequence */
    Ti_Ser_Data_Power_Init();
    while(i--);
    Ti_Ser_Data_PDB_Init();
    /* enable interrupt  pin */
    Ti_Ser_Data_Interrupt_Init();
    i=0x1F40;/* delay 1ms */
    while(i--);
    i=0x7F1200;/* delay 1s */
    while(i--);
    /* Init I2C */
    Ti_Ser_Data_I2C_Init();
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
UINT8 Ti_Ser_Data_I2C_Write(UINT8 *send_buf, UINT8 buf_size)
{
    UINT32 i;
    UINT8 u8ErrorStatus;

    if( (NULL==send_buf) || (1>buf_size))
        return I2C_ERROR_NULL;

    /* set write bit */
    send_buf[0]	 = send_buf[0]&(0xFE|I2C_WRITE);
    
    I2C_IntDisable(TI_SER_I2C);

	/* send start signals to bus */
    u8ErrorStatus = I2C_Start(TI_SER_I2C);

	/* if no error occur, received the correct ack from slave
			continue to send data to slave
		*/
    if( u8ErrorStatus == I2C_ERROR_NULL )
    {
        for(i=0;i<buf_size;i++)
        {
            u8ErrorStatus = I2C_WriteOneByte(TI_SER_I2C,send_buf[i]);
            if( u8ErrorStatus != I2C_ERROR_NULL )
            {
                break;
            }
        }
    }

	 /* send stop signals to bus */
    u8ErrorStatus = I2C_Stop(TI_SER_I2C);

    I2C_IntEnable(TI_SER_I2C);

    return u8ErrorStatus;
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
UINT8 Ti_Ser_Data_I2C_Read(UINT8 dev_add, UINT8 reg_add, UINT8 *read_data)
{
    UINT8 u8ErrorStatus;

    I2C_IntDisable(TI_SER_I2C);

	/* send start signals to bus */
    u8ErrorStatus = I2C_Start(TI_SER_I2C);
    
    /* Adress + w */
    u8ErrorStatus = I2C_WriteOneByte(TI_SER_I2C,dev_add&(0xFE|I2C_WRITE));
    /* Register address */
    u8ErrorStatus = I2C_WriteOneByte(TI_SER_I2C,reg_add);
    /* Restart */
    u8ErrorStatus =I2C_RepeatStart(TI_SER_I2C);
    /* Adress + r*/
    u8ErrorStatus = I2C_WriteOneByte(TI_SER_I2C,dev_add|I2C_READ);
    
    /* dummy read one byte to switch to Rx mode */
    u8ErrorStatus = I2C_ReadOneByte(TI_SER_I2C,read_data,I2C_SEND_ACK);

    u8ErrorStatus = I2C_ReadOneByte(TI_SER_I2C,read_data,I2C_SEND_NACK);

	 /* send stop signals to bus */
    u8ErrorStatus = I2C_Stop(TI_SER_I2C);

    I2C_IntEnable(TI_SER_I2C);

    return u8ErrorStatus;
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Ti_Ser_EDID_Cfg_Init(void)
{
#if 0
    /* for GM 8inch project */
    UINT8 EDID_data[] = {0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x09,0xE9, /* 1-10 */
                          0x26,0x77,0x45,0x54,0x00,0x00,0x26,0x16,0x01,0x03, /* 11-20 */
                          0x80,0x30,0x1B,0x78,0x2A,0x35,0x81,0xA6,0x56,0x48, /* 21-30 */
                          0x9A,0x24,0x12,0x50,0x54,0xA1,0x08,0x00,0x81,0x80, /* 31-40 */
                          0x95,0x00,0xB3,0x00,0x16,0x01,0x01,0x01,0x01,0x01, /* 41-50 */
                          0x01,0x01,0x01,0x01,0x2C,0x1A,0x00,0x3C,0x50,0x00, /* 51-60 */
                          0x0A,0x30,0x0A,0x22,0x62,0x00,0xDC,0x0B,0x11,0x00, /* 61-70 */
                          0x00,0x1A,0x00,0x00,0x00,0xFF,0x00,0x42,0x4F,0x49, /* 71-80 */
                          0x30,0x30,0x30,0x30,0x30,0x31,0x0A,0x20,0x20,0x20, /* 81-90 */
                          0x00,0x00,0x00,0xFD,0x00,0x38,0x4C,0x1E,0x37,0x0B, /* 91-100 */
                          0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00, /* 101-110 */
                          0x00,0xFC,0x00,0x31,0x39,0x32,0x30,0x58,0x37,0x32, /* 111-120 */
                          0x30,0x0A,0x20,0x20,0x20,0x20,0x00,0xB4,0x00,0x00, /* 121-130 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 131-140 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 141-150 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 151-160 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 161-170 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 171-180 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 181-190 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 191-200 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 201-210 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 211-220 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 221-230 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 231-240 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 241-250 */
                          0x00,0x00,0x00,0x00,0x00,0x00};/* 251-256 */
#endif
#if 1
#if 0
    /* for 15inch of 27.3inch */
    UINT8 EDID_data[] = {0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x09,0xE9, /* 1-10 */
                         0x26,0x77,0x45,0x54,0x00,0x00,0x26,0x16,0x01,0x03, /* 11-20*/
                         0x80,0x30,0x1B,0x78,0x2A,0x35,0x81,0xA6,0x56,0x48, /* 21-30*/
                         0x9A,0x24,0x12,0x50,0x54,0xA1,0x08,0x00,0x81,0x80, /* 31-40*/
                         0x95,0x00,0xB3,0x00,0x16,0x01,0x01,0x01,0x01,0x01, /* 41-50*/
                         0x01,0x01,0x01,0x01,0xAB,0x17,0xB0,0x78,0x40,0xD0, /* 51-60*/
                         0x2D,0x20,0x68,0x0C,0x52,0x04,0x69,0x0B,0x11,0x00, /* 61-70*/
                         0x00,0x18,0x00,0x00,0x00,0xFF,0x00,0x42,0x4F,0x49, /* 71-80*/
                         0x30,0x30,0x30,0x30,0x30,0x31,0x0A,0x20,0x20,0x20, /* 81-90*/
                         0x00,0x00,0x00,0xFD,0x00,0x38,0x4C,0x1E,0x37,0x0B, /* 91-100*/
                         0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00, /* 101-110*/
                         0x00,0xFC,0x00,0x31,0x39,0x32,0x30,0x58,0x37,0x32, /* 111-120*/
                         0x30,0x0A,0x20,0x20,0x20,0x20,0x00,0xB2,0x00,0x00, /* 121-130*/
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 131-140 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 141-150 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 151-160 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 161-170 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 171-180 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 181-190 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 191-200 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 201-210 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 211-220 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 221-230 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 231-240 */
                         0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 241-250 */
                         0x00,0x00,0x00,0x00,0x00,0x00};/* 251-256 */
#else
/* for 15inch of 27.3inch */
UINT8 EDID_data[] = {0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x09,0xE9, /* 1-10 */
                     0x26,0x77,0x45,0x54,0x00,0x00,0x26,0x16,0x01,0x03, /* 11-20*/
                     0x80,0x30,0x1B,0x78,0x2A,0x35,0x81,0xA6,0x56,0x48, /* 21-30*/
                     0x9A,0x24,0x12,0x50,0x54,0xA1,0x08,0x00,0x81,0x80, /* 31-40*/
                     0x95,0x00,0xB3,0x00,0x16,0x01,0x01,0x01,0x01,0x01, /* 41-50*/
                     0x01,0x01,0x01,0x01,0x33,0x17,0x60,0x78,0x90,0xD0, /* 51-60*/
                     0x2D,0x20,0x68,0x0C,0x52,0x04,0x69,0x0B,0x11,0x00, /* 61-70*/
                     0x00,0x18,0x00,0x00,0x00,0xFF,0x00,0x42,0x4F,0x49, /* 71-80*/
                     0x30,0x30,0x30,0x30,0x30,0x31,0x0A,0x20,0x20,0x20, /* 81-90*/
                     0x00,0x00,0x00,0xFD,0x00,0x38,0x4C,0x1E,0x37,0x0B, /* 91-100*/
                     0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00, /* 101-110*/
                     0x00,0xFC,0x00,0x31,0x39,0x32,0x30,0x58,0x37,0x32, /* 111-120*/
                     0x30,0x0A,0x20,0x20,0x20,0x20,0x00,0xD5,0x00,0x00, /* 121-130*/
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 131-140 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 141-150 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 151-160 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 161-170 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 171-180 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 181-190 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 191-200 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 201-210 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 211-220 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 221-230 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 231-240 */
                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 241-250 */
                     0x00,0x00,0x00,0x00,0x00,0x00};/* 251-256 */
#endif
#else
    /* for 12.3inch of 27.3inch */
    UINT8 EDID_data[] = {0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x09,0xE9, /* 1-10 */
                          0x26,0x77,0x45,0x54,0x00,0x00,0x26,0x16,0x01,0x03, /* 11-20*/
                          0x80,0x30,0x1B,0x78,0x2A,0x35,0x81,0xA6,0x56,0x48, /* 21-30*/
                          0x9A,0x24,0x12,0x50,0x54,0xA1,0x08,0x00,0x81,0x80, /* 31-40*/
                          0x95,0x00,0xB3,0x00,0x16,0x01,0x01,0x01,0x01,0x01, /* 41-50*/
                          0x01,0x01,0x01,0x01,0xAB,0x17,0xC0,0x78,0x30,0xD0, /* 51-60*/
                          0x2D,0x20,0x68,0x0C,0x52,0x04,0x69,0x0B,0x11,0x00, /* 61-70*/
                          0x00,0x1E,0x00,0x00,0x00,0xFF,0x00,0x42,0x4F,0x49, /* 71-80*/
                          0x30,0x30,0x30,0x30,0x30,0x31,0x0A,0x20,0x20,0x20, /* 81-90*/
                          0x00,0x00,0x00,0xFD,0x00,0x38,0x4C,0x1E,0x37,0x0B, /* 91-100*/
                          0x00,0x0A,0x20,0x20,0x20,0x20,0x20,0x20,0x00,0x00, /* 101-110*/
                          0x00,0xFC,0x00,0x31,0x39,0x32,0x30,0x58,0x37,0x32, /* 111-120*/
                          0x30,0x0A,0x20,0x20,0x20,0x20,0x00,0xAC,0x00,0x00, /* 121-130*/
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 131-140 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, /* 141-150 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 151-160 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 161-170 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 171-180 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 181-190 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 191-200 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 201-210 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 211-220 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 221-230 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 231-240 */
                          0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,/* 241-250 */
                          0x00,0x00,0x00,0x00,0x00,0x00};/* 251-256 */
#endif
    UINT32 EDID_size = sizeof(EDID_data); 
    UINT32 i;
    TI_SERIALIZE_I2C_SET    send_data={0};
    UINT8 receive_buf;
        
    Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x49,&receive_buf); 

    for(i=0;i<EDID_size;i++)
    {
        send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
        send_data.register_address = 0x4B;
        send_data.byte[0] = EDID_data[i];
        Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));
    }
#if 0
    send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
    send_data.register_address = 0x49;
    send_data.byte[0] = 0x00;
    Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

    for(i=0;i<EDID_size;i++)
    {
        send_data.device_address = TI_SER_I2C_DEV_ADDRESS;
        send_data.register_address = 0x48;
        send_data.byte[0] = 0x0F;
        Ti_Ser_Data_I2C_Write((UINT8 *)&send_data, sizeof(TI_SERIALIZE_I2C_SET));

        Ti_Ser_Data_I2C_Read(TI_SER_I2C_DEV_ADDRESS,0x4B,&receive_buf); 

        if(receive_buf != EDID_data[i])
            break;
    }
#endif    
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
UINT8 Ti_Ser_Data_Interrupt_Read(void)
{
    return GPIO_READ_INPUT(PTD,PTD2);
}

/***********************************************************************************************
*
* @brief    deserialize_if_init() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Ti_Ser_Data_Cmd_make(TOUCH_COORDINATE_BUF *touch_data)
{
    #define COMMAND_DEV_ADDR        0x6A
    /*Event codes*/
    #define EVENT_CODE_NO_EVENT     0x00/* No specific event has occured */   
    #define EVENT_CODE_MOVE         0x01/*the touch position has changed*/
    #define EVENT_CODE_UNSUP        0x02/*the touch has just been unsupppressed by 
                                                                        the touch suppression features of other objects*/
    #define EVENT_CODE_SUP          0x03/*the touch has been suppressed by 
                                                                        the touch suppression features of other objects*/
    #define EVENT_CODE_DOWN         0x04/*the touch has just come within range of the sensor*/
    #define EVENT_CODE_UP           0x05/*the touch has just left the range of the sensor*/
    #define EVENT_CODE_UNSUPSUP     0x06/*Both UNSUP and SUP events have occurred(in either order)*/
    #define EVENT_CODE_UNSUPUP      0x07/*Both UNSUP and UP events have occurred(in either order)*/
    #define EVENT_CODE_DOWNSUP      0x08/*Both DOWN and SUP events have occurred(in either order)*/
    #define EVENT_CODE_DOWNUP       0x09/*Both DOWN and UP events have occurred(in either order)*/
    /*touch type*/
    #define TOUCH_TYPE_RESERVED         0x00/*Reserved for future use*/
    #define TOUCH_TYPE_FINGER           0x01/*the touch is considered to be a finger that is contacting the screen*/
    #define TOUCH_TYPE_PASSIVE_STYLUS  0x02/*the touch is a passive stylus*/
    #define TOUCH_TYPE_HOVERING_FINGER 0x04/*the touch is a hovering finger*/
    #define TOUCH_TYPE_GLOVE            0x05/*the touch is a glove touch. Note that touches will be reported as GLOVE events only
                                                                                if the CTRL GLOVERPTEN bit in the Glove Detection T78 object is set to 1;
                                                                                otherwise they will be reported as FINGER events*/
    #define TOUCH_TYPE_LARGE_TOUCH      0x06/*the touch is a suppressed large touch. When the touch Suppression T42 CFG SUPTCHRPTEN
                                                                                bit is set, a touch that would normally be suppressed is reported. When SUPTCHRPTEN is cleared,
                                                                                the suppression method returns to normal behavior.*/
    UINT8   cmd_data[13]={0};
    UINT8   cmd_data_postion=0;
    UINT8   temp_touch[6]={0};

    if( (touch_data->read_pos+1)%COORDINATE_BUF_NUMBER == touch_data->write_pos)
        return;

    /* start to communication with deserializer */
    while((touch_data->read_pos+1)%COORDINATE_BUF_NUMBER != touch_data->write_pos)
    {
        touch_data->read_pos=(touch_data->read_pos+1)%COORDINATE_BUF_NUMBER;
        memcpy(temp_touch, touch_data->buf[touch_data->read_pos],sizeof(touch_data->buf[touch_data->read_pos]));
        memset(cmd_data, 0x00, sizeof(cmd_data));
        /*       t[0] = 0x79    sync
                        t[1] = 0x6A   Device Adress 
                        t[2] = 0x02   Register or information type
                        t[3]             length of the follow data
                        t[4]             report ID
                        t[5]             touch type
                        t[6]             event codes
                        t[7] = 0x00   LSB of x coordinate
                        t[8] = 0x00   MSB of x coordinate
                        t[9] = 0x00   LSB of y coordinate
                        t[10] = 0x00  MSB of y coordinate
                        t[11] = 0x6A   end of data
                        t[12] = 0x6A   end of data   */
        // make the header of packet
        cmd_data[cmd_data_postion]=COMMAND_SYNC;
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=COMMAND_DEV_ADDR;
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=COMMAND_ID_TOUCH;
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=0x00; //temp set ,after adding message, modify it
        
        /*report ID*/
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=temp_touch[0];
        /*touch type*/
        cmd_data_postion++;
        switch((temp_touch[1]&0x70)>>0x04)
        {
            case TOUCH_TYPE_FINGER:
                cmd_data[cmd_data_postion] = TOUCH_TYPE_FINGER;
                break;
            case TOUCH_TYPE_PASSIVE_STYLUS:
                cmd_data[cmd_data_postion] = TOUCH_TYPE_PASSIVE_STYLUS;
                break;
            case TOUCH_TYPE_HOVERING_FINGER:
                cmd_data[cmd_data_postion] = TOUCH_TYPE_HOVERING_FINGER;
                break;
            case TOUCH_TYPE_GLOVE:
                cmd_data[cmd_data_postion] = TOUCH_TYPE_GLOVE;
                break;
            case TOUCH_TYPE_LARGE_TOUCH:
                cmd_data[cmd_data_postion] = TOUCH_TYPE_LARGE_TOUCH;
                break;
            case TOUCH_TYPE_RESERVED:
            default:
                cmd_data[cmd_data_postion] = TOUCH_TYPE_RESERVED;
                break;
        }
        /* event codes */
        cmd_data_postion++;
        switch(temp_touch[1]&0x0F)
        {
            case EVENT_CODE_DOWN:
                cmd_data[cmd_data_postion] = EVENT_CODE_DOWN;
                break;
            case EVENT_CODE_UP:
                cmd_data[cmd_data_postion] = EVENT_CODE_UP;
                break;
            case EVENT_CODE_MOVE:
                cmd_data[cmd_data_postion] = EVENT_CODE_MOVE;
                break;
            case EVENT_CODE_UNSUP:
                cmd_data[cmd_data_postion] = EVENT_CODE_UNSUP;
                break;
            case EVENT_CODE_SUP:
                cmd_data[cmd_data_postion] = EVENT_CODE_SUP;
                break;
            case EVENT_CODE_UNSUPSUP:
                cmd_data[cmd_data_postion] = EVENT_CODE_UNSUPSUP;
                break;
            case EVENT_CODE_UNSUPUP:
                cmd_data[cmd_data_postion] = EVENT_CODE_UNSUPUP;
                break;
            case EVENT_CODE_DOWNSUP:
                cmd_data[cmd_data_postion] = EVENT_CODE_DOWNSUP;
                break;
            case EVENT_CODE_DOWNUP:
                cmd_data[cmd_data_postion] = EVENT_CODE_DOWNUP;
                break;
            case EVENT_CODE_NO_EVENT:
            default:
                cmd_data[cmd_data_postion] = EVENT_CODE_NO_EVENT;
                break;
        }
        /* x position */
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=temp_touch[4];
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=temp_touch[5];
        /* y position */
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=temp_touch[2];
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=temp_touch[3];
        cmd_data[3] += 7;
        
        // make the end of packet
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=COMMAND_END_DATA;
        cmd_data_postion++;
        cmd_data[cmd_data_postion]=COMMAND_END_DATA;
        cmd_data[3] += 2;
        
        Ti_Ser_Ctrl_Data_Send(cmd_data, cmd_data_postion+1);
    }
    
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Data_Power_Init(void)
{
    CONFIG_PIN_AS_GPIO(PTB,PTB0,OUTPUT); /* Configure 1V8_EN(PTB0) as an output */
    OUTPUT_SET(PTB,PTB0);
    CONFIG_PIN_AS_GPIO(PTB,PTB1,OUTPUT); /* Configure 1V1(PTB1) as an output */
    OUTPUT_SET(PTB,PTB1);
    CONFIG_PIN_AS_GPIO(PTB,PTB2,OUTPUT); /* Configure 1V2(PTB2) as an output */
    OUTPUT_SET(PTB,PTB2);
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Data_PDB_Init(void)
{
    CONFIG_PIN_AS_GPIO(PTD,PTD4,OUTPUT); /* Configure 1V2(PTB2) as an output */
    OUTPUT_SET(PTD,PTD4);
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Data_Interrupt_Init(void)
{
    CONFIG_PIN_AS_GPIO(PTD,PTD2,INPUT); /* Configure REM_INTB as an input */
    GPIO_ENABLE_INPUT(PTD,PTD2);
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Ti_Ser_Data_I2C_Init(void)
{
    I2C_ConfigType  I2C_Config = {{0}};

    /* Initialize I2C module with interrupt mode */
    I2C_Config.u16Slt = 0x0000;
//    I2C_Config.u16F = 0x0090; /* Baud rate at 100Kbps, MULT=4,  SCL divider=48, 20M/(MULT*divider) */
    I2C_Config.u16F = 0x0010; /* Baud rate at 400Kbps, MULT=1,  SCL divider=48, 20M/(MULT*divider) */
    I2C_Config.sSetting.bMSTEn = 1;/* master mode */
    I2C_Config.sSetting.bIntEn = 0;
    I2C_Config.sSetting.bI2CEn = 1;

    I2C_Init(TI_SER_I2C, &I2C_Config);
}

#endif
