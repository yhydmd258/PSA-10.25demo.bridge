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
#include <stdio.h>
#include "../../Headers/GPIO.h"
#include "../../Headers/I2C.h"
#include "../../Headers/uart.h"
#include "max_serializer_data.h"
#include "max_serializer_ctrl.h"
#include "../command.h"

/**********************************************************************************************
* Global variables
**********************************************************************************************/

/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define MAX_SERIALIZE_UART          UART_1
#define COMMUNICATION_BUF_MAX       2000
/**********************************************************************************************
* Local types
**********************************************************************************************/
typedef struct
{
    UINT32 write_pos;
    UINT32 read_pos;
    UINT8 buf[COMMUNICATION_BUF_MAX];
}COMMUNICATION_BUF_STRUCT;
/**********************************************************************************************
* Local function prototypes
*********************************************************************************************/

/**********************************************************************************************
* Local variables
**********************************************************************************************/
static COMMUNICATION_BUF_STRUCT Comm_Buf;

/**********************************************************************************************
* Local functions
**********************************************************************************************/
extern void Max_Ser_Data_IRQHander(void);
static void Max_Ser_Data_I2C_Or_Uart_Init(void);
static void Max_Ser_Data_GPIO_Init(void);

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
void Max_Ser_Data_Init(void)
{
    UINT32 i=0x800000;/* delay 1s */
    Max_Ser_Data_GPIO_Init();
    while(i--);
    Max_Ser_Data_I2C_Or_Uart_Init();
    Comm_Buf.read_pos = COMMUNICATION_BUF_MAX-1;
    Comm_Buf.write_pos=0;
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Max_Ser_Data_IRQHander(void)
{
    UINT8 receive_buf;

    receive_buf=UART_GetChar(MAX_SERIALIZE_UART);

    if((Comm_Buf.write_pos+1)%COMMUNICATION_BUF_MAX != Comm_Buf.read_pos)
    {
        Comm_Buf.buf[Comm_Buf.write_pos] = receive_buf;
        Comm_Buf.write_pos = (Comm_Buf.write_pos+1)%COMMUNICATION_BUF_MAX;
    }
//    Max_Ser_Ctrl_APP_Data_Send(&buf,1);
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Max_Ser_Receive_Data_Send(void)
{

    while((Comm_Buf.read_pos+1)%COMMUNICATION_BUF_MAX != Comm_Buf.write_pos)
    {
        Max_Ser_Ctrl_APP_Data_Send(&Comm_Buf.buf[Comm_Buf.read_pos], 1);
        Comm_Buf.read_pos = (Comm_Buf.read_pos+1)%COMMUNICATION_BUF_MAX;
    }
}

#if MAX_COMMUNICATION_TYPE_I2C
/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
UINT8 Max_Ser_Data_I2C_Write(UINT8 *send_buf, UINT8 buf_size)
{
    UINT32 i;
    UINT8 u8ErrorStatus;

    if( (NULL==send_buf) || (1>buf_size))
        return I2C_ERROR_NULL;

    /* set write bit */
    send_buf[0]	 = send_buf[0]&(0xFE|I2C_WRITE);
    
    I2C_IntDisable(MAX_SER_I2C);

	/* send start signals to bus */
    u8ErrorStatus = I2C_Start(MAX_SER_I2C);

	/* if no error occur, received the correct ack from slave
			continue to send data to slave
		*/
    if( u8ErrorStatus == I2C_ERROR_NULL )
    {
        for(i=0;i<buf_size;i++)
        {
            u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,send_buf[i]);
            if( u8ErrorStatus != I2C_ERROR_NULL )
            {
                break;
            }
        }
    }

	 /* send stop signals to bus */
    u8ErrorStatus = I2C_Stop(MAX_SER_I2C);

    I2C_IntEnable(MAX_SER_I2C);

    return u8ErrorStatus;
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
UINT8 Max_Ser_Data_I2C_Read(UINT8 dev_add, UINT8 reg_add, UINT8 *read_data)
{
    UINT8 u8ErrorStatus;

    I2C_IntDisable(MAX_SER_I2C);

	/* send start signals to bus */
    u8ErrorStatus = I2C_Start(MAX_SER_I2C);
    
    /* Adress + w */
    u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,dev_add&(0xFE|I2C_WRITE));
    /* Register address */
    u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,reg_add);
    /* Restart */
    u8ErrorStatus =I2C_RepeatStart(MAX_SER_I2C);
    /* Adress + r*/
    u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,dev_add|I2C_READ);
    
    /* dummy read one byte to switch to Rx mode */
    u8ErrorStatus = I2C_ReadOneByte(MAX_SER_I2C,read_data,I2C_SEND_ACK);

    u8ErrorStatus = I2C_ReadOneByte(MAX_SER_I2C,read_data,I2C_SEND_NACK);

	 /* send stop signals to bus */
    u8ErrorStatus = I2C_Stop(MAX_SER_I2C);

    I2C_IntEnable(MAX_SER_I2C);

    return u8ErrorStatus;
}
#else
/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Max_Ser_Data_UART_Write(MAX_SERIALIZE_UART_SET head, UINT8 *data_buf, UINT8 data_size)
{
    if((NULL == data_buf)||(0==data_size))
        return;

    head.dev_addr_rw = head.dev_addr_rw&(0xFE|I2C_WRITE);
    UART_SendWait(MAX_SERIALIZE_UART, (UINT8*)&head, sizeof(MAX_SERIALIZE_UART_SET));

    UART_SendWait(MAX_SERIALIZE_UART, data_buf, data_size);
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
void Max_Ser_Data_UART_Read(MAX_SERIALIZE_UART_SET head)
{
    head.dev_addr_rw = head.dev_addr_rw&(0xFE|I2C_READ);
    UART_SendWait(MAX_SERIALIZE_UART, (UINT8*)&head, sizeof(MAX_SERIALIZE_UART_SET));
}

#endif
/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Max_Ser_Data_I2C_Or_Uart_Init(void)
{
#if MAX_COMMUNICATION_TYPE_I2C
    I2C_ConfigType  I2C_Config = {{0}};

    /* Initialize I2C module with interrupt mode */
    I2C_Config.u16Slt = 0x0000;
//    I2C_Config.u16F = 0x0090; /* Baud rate at 100Kbps, MULT=4,  SCL divider=48, 20M/(MULT*divider) */
    I2C_Config.u16F = 0x0010; /* Baud rate at 400Kbps, MULT=1,  SCL divider=48, 20M/(MULT*divider) */
    I2C_Config.sSetting.bMSTEn = 1;/* master mode */
    I2C_Config.sSetting.bIntEn = 0;
    I2C_Config.sSetting.bI2CEn = 1;

    I2C_Init(MAX_SER_I2C, &I2C_Config);
#else
    UART_ConfigType Uart_Config={{0}};

    Uart_Config.sctrl1settings.bits.bM=1;   /* 9 bit Mode */
    Uart_Config.sctrl1settings.bits.bPe=1;  /* Parity Enable */
    Uart_Config.bSbns=0;                    /* One stop bit*/
    Uart_Config.sctrl2settings.bits.bRe=1;  /* Receiver enable*/
    Uart_Config.sctrl2settings.bits.bTe=1;  /* Transmitter enable*/
    Uart_Config.sctrl2settings.bits.bRie=1; /* Receiver interrupt enable*/
    Uart_Config.u32SysClkHz = 20000000;     /* Bus clock in Hz*/
//    Uart_Config.u32Baudrate = 256000;       /* Baud rate*/
   Uart_Config.u32Baudrate = 9600;       /* Baud rate*/
//	Uart_Config.u32Baudrate = 115200; 	  /* Baud rate*/
    /*Initialization of UART module*/
    UART_SetCallback(MAX_SERIALIZE_UART, Max_Ser_Data_IRQHander);
    UART_Init(MAX_SERIALIZE_UART,&Uart_Config);
#endif
}
/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
static void Max_Ser_Data_GPIO_Init(void)
{
#if MAX_COMMUNICATION_TYPE_I2C
    CONFIG_PIN_AS_GPIO(PTF,PTF0,OUTPUT); /* Configure I2CSEL(PTF0) as an output */
    OUTPUT_SET(PTF,PTF0);
#else
    SIM_PINSEL1 |= SIM_PINSEL1_UART1PS_MASK;/* Tx:PTF3, RX:PTF2 */
    CONFIG_PIN_AS_GPIO(PTF,PTF0,OUTPUT); /* Configure I2CSEL(PTF0) as an output */
    OUTPUT_CLEAR(PTF,PTF0);
#endif
    CONFIG_PIN_AS_GPIO(PTE,PTE6,OUTPUT); /* Configure SER_PWDN(PTE6) as an output */
    OUTPUT_SET(PTE,PTE6);
    CONFIG_PIN_AS_GPIO(PTG,PTG3,INPUT);  /* Configure Maxim_Fault(PTG3) as an input */	
    GPIO_ENABLE_INPUT(PTG,PTG3);
    CONFIG_PIN_AS_GPIO(PTB,PTB0,OUTPUT); /* Configure power enable(PTB0) as an output */
    OUTPUT_SET(PTB,PTB0);
	/*clear MS pin make ser standard mode*/
	CONFIG_PIN_AS_GPIO(PTF,PTF7,OUTPUT); /* Configure MS as an output */
    OUTPUT_CLEAR(PTF,PTF7);
	CONFIG_PIN_AS_GPIO(PTF,PTF6,OUTPUT); /* Configure DEBUG pin identical MS as an output */
    OUTPUT_CLEAR(PTF,PTF6);
    /* Configure Inout */
    /* Configure GPO */
}

/***********************************************************************************************
*
* @brief    main() - Program entry function
* @param    none
* @return   none
*
************************************************************************************************/
UINT8 Max_Ser_EDID_Cfg_Init(void)
{
    UINT8 u8ErrorStatus = 0;
#if 1
    UINT8 EDID_data[]={0,  255,255,255,255,255,255,0,13,174,   /*1-10*/
                        0X36,0X10,0X00,0X00,0X00,0X00,0X1C,0X15,0X01,0X03,     /*11-20*/
                        0X80,0X16,0X0D,0X78,0X0A,0X45,0X35,0X99,0X57,0X4E,    /*21-30*/
                        0X92,0X25,0X1C,0X50,0X54,0X00,0X00,0X00,0X01,0X01,     /*31-40*/
                        1,  1,  1,  1,  1,  1,  1,  1,  1,  1,       /*41-50*/
                        1,  1,  1,  1,  0X10,0X27,0X80,0X68,0X71,0XD0,    /*51-60*/
                        0X0B,0X20,0X38,0X08,0X82,0X40,0XF3,0X5B,0X00,0X00,      /*61-70*/
                        0,  24, 0,  0,  0,  254,0,  78, 49, 48,     /*71-80*/
                        01, 66, 67, 71, 45, 76, 50, 49, 32, 32,     /*81-90*/
                        0,  0,  0,  254,0,  67, 77, 78, 10, 32,     /*91-100*/
                        32, 32, 32, 32, 32, 32, 32, 32, 0,  0,      /*101-110*/
                        0,  254,0,  78, 49, 48, 49, 66, 67, 71,     /*111-120*/
                        45, 76, 50, 49, 32, 32, 0,  0X17, 0,  0,      /*121-130*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*131-140*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*141-150*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*151-160*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*161-170*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*171-180*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*181-190*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*191-200*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*201-210*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*211-220*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*221-230*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*231-240*/
                        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,       /*241-250*/
                        0,  0,  0,  0,  0,  0};     /*251-256*/
#endif
#if 0
	UINT8 EDID_data[]={0,  255,255,255,255,255,255,0,  13, 174,	/*1-10*/
							54, 16, 0,	0,	0,	0,	28, 21,  1,  3, 	/*11-20*/
							128,22, 13, 120,10, 69, 53, 153,87, 78,    /*21-30*/
							146,37, 28, 80, 84, 0,	0,	0,	1,	1,		/*31-40*/
							1,	1,	1,	1,	1,	1,	1,	1,	1,	1,		 /*41-50*/
							1,	1,	1,	1,	76, 29, 128,42, 112,208,	/*51-60*/
							28, 32, 26, 10, 148,0,	222,125,0,	0,		/*61-70*/
							0,	24, 0,	0,	0,	254,0,	78, 49, 48, 	/*71-80*/
							49, 66, 67, 71, 45, 76, 50, 49, 32, 32, 	/*81-90*/
							0,	0,	0,	254,0,	67, 77, 78, 10, 32, 	/*91-100*/
							32, 32, 32, 32, 32, 32, 32, 32, 0,	0,		/*101-110*/
							0,	254,0,	78, 49, 48, 49, 66, 67, 71, 	/*111-120*/
							45, 76, 50, 49, 32, 32, 0,	32, 0,	0,		/*121-130*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*131-140*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*141-150*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*151-160*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*161-170*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*171-180*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*181-190*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*191-200*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*201-210*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*211-220*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*221-230*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*231-240*/
							0,	0,	0,	0,	0,	0,	0,	0,	0,	0,		 /*241-250*/
							0,	0,	0,	0,	0,	0}; 	/*251-256*/
#endif
#if MAX_COMMUNICATION_TYPE_I2C
    UINT32 EDID_size = sizeof(EDID_data); 
    UINT32 i;
//    UINT8 data=0;
    I2C_IntDisable(MAX_SER_I2C);

	/* send start signals to bus */
    u8ErrorStatus = I2C_Start(MAX_SER_I2C);

	/* if no error occur, received the correct ack from slave
			continue to send data to slave
		*/
    /* adress + w */
    u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,CMD_SER_EDID_DEV_ADRESS&(0xFE|I2C_WRITE));
    /* Register address */
    u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,0x00);
    
    if( u8ErrorStatus == I2C_ERROR_NULL )
    {
        for(i=0;i<EDID_size;i++)
        {
            u8ErrorStatus = I2C_WriteOneByte(MAX_SER_I2C,EDID_data[i]);
            if( u8ErrorStatus != I2C_ERROR_NULL )
            {
                return u8ErrorStatus;
            }
        }
    }
	 /* send stop signals to bus */
    u8ErrorStatus = I2C_Stop(MAX_SER_I2C);

    I2C_IntEnable(MAX_SER_I2C);
//    for(i=0;i<EDID_size-1;i++)
//    {
//        u8ErrorStatus = Max_Ser_Data_I2C_Read(CMD_SER_EDID_DEV_ADRESS,i,&data);
//        if(data!=EDID_data[i])
//            break;
//    }
#else
    MAX_SERIALIZE_UART_SET  ser_uart_set;
    UINT32  delay = 0x00;

    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = CMD_SER_EDID_DEV_ADRESS;
    ser_uart_set.reg_addr = 0x00;
    ser_uart_set.byte_number = 0xFF;
    Max_Ser_Data_UART_Write(ser_uart_set, EDID_data, ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);
    
    ser_uart_set.sync = COMMAND_SYNC;
    ser_uart_set.dev_addr_rw = CMD_SER_EDID_DEV_ADRESS;
    ser_uart_set.reg_addr = 0xFF;
    ser_uart_set.byte_number = 0x01;
    Max_Ser_Data_UART_Write(ser_uart_set, &(EDID_data[ser_uart_set.reg_addr]), ser_uart_set.byte_number);
    delay = 0x80000;
    while(delay--);
#endif
    return u8ErrorStatus;
}

#endif
