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
#include <string.h>
#include "APP_COMM_data.h"
#include "../../Headers/GPIO.h"
#include "../../Headers/uart.h"
/**********************************************************************************************
* External objects
**********************************************************************************************/

/**********************************************************************************************
* Global variables
**********************************************************************************************/

/**********************************************************************************************
* Constants and macros
**********************************************************************************************/
#define SER_COMM_UART          UART_1
#define APP_COMM_UART          UART_2
#define COMMUNICATION_BUF_MAX  100

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
static COMMUNICATION_BUF_STRUCT Comm_Buf={0,0};


/**********************************************************************************************
* Local functions
**********************************************************************************************/
static void APP_COMM_Data_IRQHander(void);
static void APP_COMM_Data_UART_Init(void);

/**********************************************************************************************
* Global functions
**********************************************************************************************/
/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
void APP_COMM_Data_Init(void)
{
    APP_COMM_Data_UART_Init();

}
/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
void APP_COMM_Data_Send(UINT8* send_buf, UINT32 buf_size)
{
    if(NULL != send_buf)
        UART_SendWait(APP_COMM_UART, send_buf, buf_size);
}
/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
static UINT8 num=1;
void APP_COMM_Data_SER_Task(void)
{

#ifdef DEBUG
	UINT8 buf[]={0x79,0x66,0x01,0x03,0x00,0x66,0xaa};
if(num)
{
	APP_COMM_Data_SER_Send(buf,7);
	num-=1;
}
#endif

    while(Comm_Buf.read_pos!= Comm_Buf.write_pos)
    {
        APP_COMM_Data_SER_Send(&Comm_Buf.buf[Comm_Buf.read_pos], 1);
        Comm_Buf.read_pos = (Comm_Buf.read_pos+1)%COMMUNICATION_BUF_MAX;
    }

}

/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
void APP_COMM_Data_SER_Send(UINT8* send_buf, UINT32 buf_size)
{
    if(NULL != send_buf)
        UART_SendWait(SER_COMM_UART, send_buf, buf_size);
}

/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
static void APP_COMM_Data_UART_Init(void)
{
    UART_ConfigType Uart_Config={{0}};

    Uart_Config.sctrl1settings.bits.bM=1;   /* 9 bit Mode */
    Uart_Config.sctrl1settings.bits.bPe=1;  /* Parity Enable */
    Uart_Config.bSbns=0;                    /* One stop bit*/
    Uart_Config.sctrl2settings.bits.bRe=1;  /* Receiver enable*/
    Uart_Config.sctrl2settings.bits.bTe=1;  /* Transmitter enable*/
    Uart_Config.sctrl2settings.bits.bRie=1; /* Receiver interrupt enable*/
    Uart_Config.u32SysClkHz = 20000000;     /* Bus clock in Hz*/
    Uart_Config.u32Baudrate = 256000;       /* Baud rate*/

    /*Initialization of UART module*/
    UART_SetCallback(APP_COMM_UART, APP_COMM_Data_IRQHander);
    UART_Init(APP_COMM_UART,&Uart_Config);
}

/***********************************************************************************************
*
* @brief    
* @param    none
* @return   none
*
************************************************************************************************/
static void APP_COMM_Data_IRQHander(void)
{
    UINT8 receive_buf;

    receive_buf=UART_GetChar(APP_COMM_UART);

    if((Comm_Buf.write_pos+1)%COMMUNICATION_BUF_MAX != Comm_Buf.read_pos)
    {
        Comm_Buf.buf[Comm_Buf.write_pos] = receive_buf;
        Comm_Buf.write_pos = (Comm_Buf.write_pos+1)%COMMUNICATION_BUF_MAX;
    }
}
