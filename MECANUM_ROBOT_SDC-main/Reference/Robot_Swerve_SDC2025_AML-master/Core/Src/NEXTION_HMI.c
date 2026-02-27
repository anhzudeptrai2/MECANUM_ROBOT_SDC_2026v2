#include "NEXTION_HMI.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>


uint8_t NextionAddComp(Nextion *nex, NexComp *_nexcomp, char *objectname, uint8_t __page, uint8_t __id,
					   void (*callbackFuncOnPress)(), void (*callbackFuncOnRelease)())
{
	_nexcomp->objname = (char *)malloc(strlen(objectname) + 1);
	strcpy(_nexcomp->objname, objectname);
	_nexcomp->_id = __id;
	_nexcomp->_page = __page;
	nex->_NexCompArr[nex->_NexCompCount] = _nexcomp;
	nex->_NexCompCount++;
	_nexcomp->callbackOnPress = callbackFuncOnPress;
	_nexcomp->callbackOnRelease = callbackFuncOnRelease;
	return 0;
}


uint8_t NextionInit(Nextion *nex, UART_HandleTypeDef *nextionUARTHandle)
{
	nex->nextionUARTHandle = nextionUARTHandle;
	nex->_arrCount = 0;
	nex->_pkgCount = 0;
	nex->_NexCompCount = 0;
	nex->NexTextBuff[0] = '\0';
	nex->NextTextLen = 0;
	nex->NextNumBuff = 0;
	nex->txBusy = 0;
	nex->currentTxLength = 0;
	nex->cmdQueue.head = 0;
	nex->cmdQueue.tail = 0;
	HAL_UART_Receive_IT(nex->nextionUARTHandle, &nex->_RxData, 1);
	return 0;
}


uint8_t NextionUpdate(UART_HandleTypeDef *huart, Nextion *nex)
{
	if (huart->Instance == nex->nextionUARTHandle->Instance)
	{
		nex->_RxDataArr[nex->_arrCount++] = nex->_RxData;
		if (nex->_RxData == 0xFF)
			nex->_pkgCount++;
		else
			nex->_pkgCount = 0;

		if (nex->_pkgCount == 3)
		{
			uint8_t count = 0, FFCount = 0;
			for (uint8_t i = 0; FFCount < 3; i++)
			{
				count++;
				if (nex->_RxDataArr[i] == 0xFF)
					FFCount++;
			}
			// Xử lý sự kiện touch
			if (nex->_RxDataArr[0] == NEX_RET_EVENT_TOUCH_HEAD)
			{
				for (uint8_t i = 0; i < nex->_NexCompCount; i++)
				{
					if ((nex->_RxDataArr[2] == nex->_NexCompArr[i]->_id) &&
						(nex->_RxDataArr[1] == nex->_NexCompArr[i]->_page))
					{
						if ((nex->_RxDataArr[3] == NEX_EVENT_ON_PRESS) && (nex->_NexCompArr[i]->callbackOnPress != NULL))
							nex->_NexCompArr[i]->callbackOnPress();
						if ((nex->_RxDataArr[3] == NEX_EVENT_ON_RELEASE) && (nex->_NexCompArr[i]->callbackOnRelease != NULL))
							nex->_NexCompArr[i]->callbackOnRelease();
					}
				}
			}
			// Xử lý gói dữ liệu chứa chuỗi (string)
			if (nex->_RxDataArr[0] == NEX_RET_STRING_HEAD)
			{
				nex->NextTextLen = 0;
				for (int i = 0; i < (count - 4) && i < (NEXTION_TEXT_BUFF_LEN - 1); i++)
				{
					nex->NexTextBuff[i] = nex->_RxDataArr[i + 1];
					nex->NextTextLen++;
				}
				nex->NexTextBuff[nex->NextTextLen] = '\0';
			}
			// Xử lý gói dữ liệu chứa số (number)
			if (nex->_RxDataArr[0] == NEX_RET_NUMBER_HEAD)
			{
				nex->NextNumBuff = ((uint32_t)(nex->_RxDataArr[4] << 24)) |
								   ((uint32_t)(nex->_RxDataArr[3] << 16)) |
								   (nex->_RxDataArr[2] << 8) |
								   (nex->_RxDataArr[1]);
			}
			nex->_pkgCount = 0;
			nex->_arrCount = 0;
		}
		HAL_UART_Receive_IT(nex->nextionUARTHandle, &nex->_RxData, 1);
	}
	return 0;
}


uint8_t Nextion_EnqueueCommand(Nextion *nex, const char *command)
{
	size_t len = strlen(command);
	for (size_t i = 0; i < len; i++)
	{
		uint16_t nextHead = (nex->cmdQueue.head + 1) % CMD_BUFFER_SIZE;
		if (nextHead == nex->cmdQueue.tail)
			return 1; // Buffer đầy
		nex->cmdQueue.buffer[nex->cmdQueue.head] = command[i];
		nex->cmdQueue.head = nextHead;
	}
	// Thêm 3 byte kết thúc lệnh (0xFF)
	for (int i = 0; i < 3; i++)
	{
		uint16_t nextHead = (nex->cmdQueue.head + 1) % CMD_BUFFER_SIZE;
		if (nextHead == nex->cmdQueue.tail)
			return 1;
		nex->cmdQueue.buffer[nex->cmdQueue.head] = 0xFF;
		nex->cmdQueue.head = nextHead;
	}
	return 0;
}


uint8_t NextionSendCommand(Nextion *nex, char *_command)
{
	return Nextion_EnqueueCommand(nex, _command);
}

uint8_t NextionEndCommand(Nextion *nex)
{
	return 0;
}


uint8_t NextionGetText(Nextion *nex, NexComp *comp, char *buf)
{
	char transmitBuff[NEXTION_TEXT_BUFF_LEN] = {0};
	sprintf(transmitBuff, "get %s.txt", comp->objname);
	NextionSendCommand(nex, transmitBuff);
	strncpy(buf, (char *)nex->NexTextBuff, nex->NextTextLen);
	buf[nex->NextTextLen] = '\0';
	return 0;
}

uint8_t NextionSetText(Nextion *nex, NexComp *comp, char *usertext)
{
	char transmitBuff[NEXTION_TEXT_BUFF_LEN] = {0};
	sprintf(transmitBuff, "%s.txt=\"%s\"", comp->objname, usertext);
	NextionSendCommand(nex, transmitBuff);
	return 0;
}

uint8_t NextionGetVal(Nextion *nex, NexComp *comp, int *valBuf)
{
	char transmitBuff[NEXTION_TEXT_BUFF_LEN] = {0};
	sprintf(transmitBuff, "get %s.val", comp->objname);
	NextionSendCommand(nex, transmitBuff);
	*valBuf = nex->NextNumBuff;
	return 0;
}

uint8_t NextionSetVal(Nextion *nex, NexComp *comp, int userval)
{
	char transmitBuff[NEXTION_TEXT_BUFF_LEN] = {0};
	sprintf(transmitBuff, "%s.val=%d", comp->objname, userval);
	NextionSendCommand(nex, transmitBuff);
	return 0;
}


uint8_t NextionRestartIT(Nextion *nex)
{
	HAL_UART_Receive_IT(nex->nextionUARTHandle, &nex->_RxData, 1);
	return 0;
}

uint8_t NextionStopIT(Nextion *nex)
{
	HAL_UART_AbortReceive_IT(nex->nextionUARTHandle);
	return 0;
}


void Nextion_Process(Nextion *nex)
{
	if (nex->txBusy == 0)
	{
		uint16_t pending = 0;
		if (nex->cmdQueue.head >= nex->cmdQueue.tail)
			pending = nex->cmdQueue.head - nex->cmdQueue.tail;
		else
			pending = CMD_BUFFER_SIZE - nex->cmdQueue.tail;

		if (pending > 0)
		{
			nex->txBusy = 1;
			nex->currentTxLength = pending;
			HAL_UART_Transmit_DMA(nex->nextionUARTHandle,
								  (uint8_t *)&nex->cmdQueue.buffer[nex->cmdQueue.tail],
								  pending);
		}
	}
}


void Nextion_UART_TxCpltCallback(Nextion *nex)
{
	nex->cmdQueue.tail = (nex->cmdQueue.tail + nex->currentTxLength) % CMD_BUFFER_SIZE;
	nex->txBusy = 0;
}
