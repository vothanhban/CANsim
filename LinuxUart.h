/*
 * LinuxUart.h
 *
 *  Created on: Nov 27, 2014
 *      Author: nvthanh
 */

#ifndef LINUXUART_H_
#define LINUXUART_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#ifdef __cplusplus
extern "C" {
#endif
#define UART_BUFF_SZ 30
typedef struct linux_uart_data{
	int fd;
	uint8_t  rxData[UART_BUFF_SZ];
	uint8_t  txData[UART_BUFF_SZ];
	uint16_t rxSz;
	uint32_t baudrate;
	void (*irq_handler)(int status);
}linux_uart_data;

linux_uart_data *					linux_uart_open (uint8_t * port /*device node*/);
int  								linux_uart_close (struct linux_uart_data * pud);
int 								linux_uart_flush (struct linux_uart_data * pud);
int 								linux_uart_set_speed (struct linux_uart_data * pud,uint32_t baudrate);
size_t			 					linux_uart_write (struct linux_uart_data * pud,uint8_t * data,size_t szdata);
size_t	 							linux_uart_read (struct linux_uart_data * pud,uint8_t * res,size_t szexp,uint32_t timeout);
void								linux_uart_enable_irq(struct linux_uart_data * pud,void (*irq_handler)(int status));
void								linux_uart_disable_irq(struct linux_uart_data * pud);

#ifdef __cplusplus
}//extern "C" {
#endif




#endif /* LINUXUART_H_ */
