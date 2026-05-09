/* Includes ------------------------------------------------------------------*/
#include "Ethernet.h"
#include "main.h"

/* External variables --------------------------------------------------------*/
extern uint8_t rxPool[RX_BUF_CNT][RX_BUF_SIZE];
extern BufStatus rxBufStatus[RX_BUF_CNT];
extern uint8_t rxQueueSize;
extern uint32_t counterRx;

/* Overridden function definitions -------------------------------------------*/
void HAL_ETH_RxAllocateCallback(uint8_t **buff) {
	static uint32_t allocIdx = 0;

	for (uint32_t i = 0; i < RX_BUF_CNT; i++) {
		if (rxBufStatus[allocIdx] == BUF_FREE) {
			rxBufStatus[allocIdx] = BUF_OWNED_DMA;
			*buff = rxPool[allocIdx];
			allocIdx = (allocIdx + 1) % RX_BUF_CNT;
			return;
		}
		allocIdx = (allocIdx + 1) % RX_BUF_CNT;
	}

	*buff = NULL;
}

void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length) {
	*pStart = buff;
	*pEnd   = buff + Length;
}

void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth) {
	counterRx++;

	uint8_t *rxBuf;
	if (HAL_ETH_ReadData(heth, (void**)&rxBuf) == HAL_OK) {
		uint32_t idx = ((uint32_t)rxBuf - (uint32_t)rxPool) / RX_BUF_SIZE;

		rxBufStatus[idx] = BUF_OWNED_CPU;

		__disable_irq();
		rxQueueSize++;
		__enable_irq();
	}
}
