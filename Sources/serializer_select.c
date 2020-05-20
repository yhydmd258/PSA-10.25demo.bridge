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
#include "serializer_select.h"
#include "../Headers/GPIO.h"
/**********************************************************************************************
* External objects
**********************************************************************************************/

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
void Ser_Select_Init(void)
{
    CONFIG_PIN_AS_GPIO(PTF,PTF6,INPUT); /* Configure the serializer select pin */
    GPIO_ENABLE_INPUT(PTF,PTF6);
}

/***********************************************************************************************
*
* @brief    
* @param    none
* @return   max: 0,  Ti: 1
*
************************************************************************************************/
UINT8 Ser_Select_Read(void)
{
    return GPIO_READ_INPUT(PTF,PTF6);
}
