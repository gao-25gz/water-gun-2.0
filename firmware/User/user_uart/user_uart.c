#include "user_uart.h"
#include "gimbal_control.h"

uint8_t usart1SendBuf[USART_SEND_BUF_SIZE + 1];
uint8_t usart1RecvBuf[USART_RECV_BUF_SIZE + 1];
RingBufferTypeDef usart1SendRingBuf;
RingBufferTypeDef usart1RecvRingBuf;
Usart_DataTypeDef FSUS_usart1;
uint8_t rc1;

void User_Uart_Init(UART_HandleTypeDef *huartx)
{
	// 创建收发环形缓冲区。
	RingBuffer_Init(&usart1SendRingBuf, USART_SEND_BUF_SIZE, usart1SendBuf);
	RingBuffer_Init(&usart1RecvRingBuf, USART_RECV_BUF_SIZE, usart1RecvBuf);
	// 初始化舵机串口结构。
	FSUS_usart1.recvBuf = &usart1RecvRingBuf;
	FSUS_usart1.sendBuf = &usart1SendRingBuf;
	FSUS_usart1.huartX = huartx;

	/* PA9 是共享的 DATA/S 总线，先进入接收模式，
	 * 让舵机回包进入原有的环形缓冲和协议层。 */
	HAL_HalfDuplex_EnableReceiver(FSUS_usart1.huartX);
	HAL_UART_Receive_IT(FSUS_usart1.huartX, (uint8_t *)&rc1, 1);
}

// 发送环形缓冲中的全部数据。
void Usart_SendAll(Usart_DataTypeDef *usart)
{
	uint8_t value;
	uint8_t halfDuplex = (usart->huartX->Instance == USART1);

	if (RingBuffer_GetByteUsed(usart->sendBuf) == 0U)
	{
		return;
	}

	if (halfDuplex)
	{
		/* 发送前停止单字节接收，取得 PA9 总线控制权。 */
		HAL_UART_AbortReceive(usart->huartX);
		HAL_HalfDuplex_EnableTransmitter(usart->huartX);
	}

	while (RingBuffer_GetByteUsed(usart->sendBuf))
	{
		value = RingBuffer_Pop(usart->sendBuf);
		/* HAL_UART_Transmit 会等待 TC，最后一个停止位完成后
		 * 才把总线交还给舵机。 */
		HAL_UART_Transmit(usart->huartX, &value, 1, 10);
	}

	if (halfDuplex)
	{
		HAL_HalfDuplex_EnableReceiver(usart->huartX);
		HAL_UART_Receive_IT(usart->huartX, (uint8_t *)&rc1, 1);
	}
}
// 串口接收中断回调。
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t ucTemp;
	if (huart->Instance == FSUS_usart1.huartX->Instance)
	{
		ucTemp = rc1;
		// 中断中只存入缓冲区，不在中断里解析协议。
		RingBuffer_Push(FSUS_usart1.recvBuf, ucTemp);
		HAL_UART_Receive_IT(FSUS_usart1.huartX, (uint8_t *)&rc1, 1);
	}
	else if (huart->Instance == USART2)
	{
		Gimbal_UartRxCpltCallback(huart);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)
	{
		/* 半双工单线收发切换时偶发 ORE 溢出，清标志并恢复接收，
		 * 否则 ORE 不清会让错误中断一直挂起、卡死主循环。 */
		__HAL_UART_CLEAR_OREFLAG(huart);
		HAL_HalfDuplex_EnableReceiver(huart);
		HAL_UART_Receive_IT(huart, &rc1, 1);
	}
	else if (huart->Instance == USART2)
	{
		Gimbal_RestartRx();
	}
}
