#ifndef  _COMMAND_H_
#define _COMMAND_H_

#ifdef __cplusplus
extern "C" {
#endif

/*command start*/
#define COMMAND_SYNC             0x79
#define CMD_SER_DEV_ADDR         0x80
#define CMD_SER_EDID_DEV_ADRESS 0xFE
#define CMD_DESER_DEV_ADDR       0x90
/* command type ID */
#define COMMAND_ID_BACKLIGHT    0x01    /* receive */
#define COMMAND_ID_TOUCH        0x02    /* send touch message */
#define COMMAND_ID_LED_FAULT    0x03    /* send LED fault message */
#define COMMAND_ID_LCD_FAULT    0x04    /* send LCD fault message */

/*command end*/
#define COMMAND_END_DATA        0x6A

/* state */
#define ERROR_NULL              0x00
#define ERROR                   0x01

#ifdef __cplusplus
}
#endif

#endif/* _COMMAND_H_ */
