#ifndef INC_NEXTION_H_
#define INC_NEXTION_H_

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"

#include "stm32h7xx_hal.h"		/* Import HAL library */
#include "stm32h7xx_hal_uart.h" /* Import HAL library */

/*
 * Defines
 */
#define NEXTION_TIMEOUT 250
#define NEXTION_MAX_BUFF_LEN 96
#define NEXTION_TEXT_BUFF_LEN 64
#define NEXTION_MAX_COMP_COUNT 200
#define CMD_BUFFER_SIZE 256

#define NEX_RET_CMD_FINISHED (0x01)
#define NEX_RET_EVENT_LAUNCHED (0x88)
#define NEX_RET_EVENT_UPGRADED (0x89)
#define NEX_RET_EVENT_TOUCH_HEAD (0x65)
#define NEX_RET_EVENT_POSITION_HEAD (0x67)
#define NEX_RET_EVENT_SLEEP_POSITION_HEAD (0x68)
#define NEX_RET_CURRENT_PAGE_ID_HEAD (0x66)
#define NEX_RET_STRING_HEAD (0x70)
#define NEX_RET_NUMBER_HEAD (0x71)
#define NEX_RET_INVALID_CMD (0x00)
#define NEX_RET_INVALID_COMPONENT_ID (0x02)
#define NEX_RET_INVALID_PAGE_ID (0x03)
#define NEX_RET_INVALID_PICTURE_ID (0x04)
#define NEX_RET_INVALID_FONT_ID (0x05)
#define NEX_RET_INVALID_BAUD (0x11)
#define NEX_RET_INVALID_VARIABLE (0x1A)
#define NEX_RET_INVALID_OPERATION (0x1B)
#define NEX_EVENT_ON_PRESS (0x01)
#define NEX_EVENT_ON_RELEASE (0x00)

/*
 * NexComp Struct
 */
typedef struct
{
	uint8_t _page, _id;
	void (*callbackOnPress)();
	void (*callbackOnRelease)();
	char *objname;
} NexComp;

/*
 * Command Queue Structure 
 */
typedef struct
{
	char buffer[CMD_BUFFER_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
} CommandQueue;

/*
 * Nextion Struct
 */
typedef struct
{
	UART_HandleTypeDef *nextionUARTHandle;

	uint8_t _RxDataArr[NEXTION_MAX_BUFF_LEN], _RxData, _arrCount, _pkgCount;

	NexComp *_NexCompArr[NEXTION_MAX_COMP_COUNT];
	uint8_t _NexCompCount;

	uint8_t NexTextBuff[NEXTION_TEXT_BUFF_LEN], NextTextLen;
	int32_t NextNumBuff;

	CommandQueue cmdQueue;
	volatile uint8_t txBusy;		  
	volatile uint16_t currentTxLength; 
} Nextion;

/*
 * Library User functions
 */
uint8_t NextionAddComp(Nextion *nex, NexComp *_nexcomp, char *objectname, uint8_t __page, uint8_t __id,
					   void (*callbackFuncOnPress)(), void (*callbackFuncOnRelease)());
uint8_t NextionUpdate(UART_HandleTypeDef *huart, Nextion *nex);
uint8_t NextionInit(Nextion *nex, UART_HandleTypeDef *nextionUARTHandle);
uint8_t NextionGetText(Nextion *nex, NexComp *comp, char *buf);
uint8_t NextionSetText(Nextion *nex, NexComp *comp, char *usertext);
uint8_t NextionGetVal(Nextion *nex, NexComp *comp, int *valBuf);
uint8_t NextionSetVal(Nextion *nex, NexComp *comp, int userval);

/*
 * Library Low Level Functions
 */
uint8_t NextionSendCommand(Nextion *nex, char *_command);
uint8_t NextionEndCommand(Nextion *nex);
uint8_t NextionRestartIT(Nextion *nex);
uint8_t NextionStopIT(Nextion *nex);

/*
 * Non-blocking processing and DMA TX callback functions
 */
void Nextion_Process(Nextion *nex);
void Nextion_UART_TxCpltCallback(Nextion *nex);

#endif /* INC_NEXTION_H_ */
