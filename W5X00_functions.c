/*
 * W5X00_functions.c
 *
 *  Created on: May 4, 2020
 *      Author: BER83WI
 */

#include <wizchip_conf.h>
#include "pin.h"
#include "socket.h"
#include "string.h"
#include <stdio.h>

#define SEPARATOR            "=============================================\r\n"
#define WELCOME_MSG  		 "Welcome to STM32Nucleo Ethernet configuration\r\n"
#define NETWORK_MSG  		 "Network configuration:\r\n"
#define IP_MSG 		 		 "  IP ADDRESS:  %d.%d.%d.%d\r\n"
#define NETMASK_MSG	         "  NETMASK:     %d.%d.%d.%d\r\n"
#define GW_MSG 		 		 "  GATEWAY:     %d.%d.%d.%d\r\n"
#define MAC_MSG		 		 "  MAC ADDRESS: %x:%x:%x:%x:%x:%x\r\n"
#define GREETING_MSG 		 "Well done guys! Welcome to the IoT world. Bye!\r\n"
#define CONN_ESTABLISHED_MSG "Connection established with remote IP: %d.%d.%d.%d:%d\r\n"
#define SENT_MESSAGE_MSG	 "Sent a message. Let's close the socket!\r\n"
#define WRONG_RETVAL_MSG	 "Something went wrong; return value: %d\r\n"
#define WRONG_STATUS_MSG	 "Something went wrong; STATUS: %d\r\n"
#define LISTEN_ERR_MSG		 "LISTEN Error!\r\n"

#define PRINT_STR(msg) do  {										\
		HAL_UART_Transmit(huart2, (uint8_t*)msg, strlen(msg), 100);		\
} while(0)

#define PRINT_HEADER() do  {													\
		HAL_UART_Transmit(huart2, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
		HAL_UART_Transmit(huart2, (uint8_t*)WELCOME_MSG, strlen(WELCOME_MSG), 100);	\
		HAL_UART_Transmit(huart2, (uint8_t*)SEPARATOR, strlen(SEPARATOR), 100);		\
} while(0)

#define PRINT_NETINFO(netInfo) do { 																					\
		HAL_UART_Transmit(huart2, (uint8_t*)NETWORK_MSG, strlen(NETWORK_MSG), 100);											\
		sprintf(msg, MAC_MSG, netInfo.mac[0], netInfo.mac[1], netInfo.mac[2], netInfo.mac[3], netInfo.mac[4], netInfo.mac[5]);\
		HAL_UART_Transmit(huart2, (uint8_t*)msg, strlen(msg), 100);															\
		sprintf(msg, IP_MSG, netInfo.ip[0], netInfo.ip[1], netInfo.ip[2], netInfo.ip[3]);										\
		HAL_UART_Transmit(huart2, (uint8_t*)msg, strlen(msg), 100);															\
		sprintf(msg, NETMASK_MSG, netInfo.sn[0], netInfo.sn[1], netInfo.sn[2], netInfo.sn[3]);								\
		HAL_UART_Transmit(huart2, (uint8_t*)msg, strlen(msg), 100);															\
		sprintf(msg, GW_MSG, netInfo.gw[0], netInfo.gw[1], netInfo.gw[2], netInfo.gw[3]);										\
		HAL_UART_Transmit(huart2, (uint8_t*)msg, strlen(msg), 100);															\
} while(0)

SPI_HandleTypeDef* hspi = NULL;
UART_HandleTypeDef* huart2 = NULL;
pin_t W5X00CS;
uint8_t retVal, sockStatus;
int16_t rcvLen;
uint8_t rcvBuf[20], bufSize[] = {2, 2, 2, 2};
char msg[60];

void W5X00cs_sel() {
	HAL_GPIO_WritePin(W5X00CS.Port, W5X00CS.num, GPIO_PIN_RESET);
}

void W5X00cs_desel() {
	HAL_GPIO_WritePin(W5X00CS.Port, W5X00CS.num, GPIO_PIN_SET);
}

uint8_t W5X00spi_rb(void) {
	uint8_t rbuf;
	HAL_SPI_Receive(hspi, &rbuf, 1, 0xFFFFFFFF);
	return rbuf;
}

void W5X00spi_wb(uint8_t tbuf) {
	HAL_SPI_Transmit(hspi, &tbuf, 1, 0xFFFFFFFF);
}

void setW5X00Spi(SPI_HandleTypeDef* newhspi, GPIO_TypeDef* CSPort, uint16_t CSNum) {
	hspi = newhspi;
	W5X00CS.Port = CSPort;
	W5X00CS.num = CSNum;

	reg_wizchip_cs_cbfunc(W5X00cs_sel, W5X00cs_desel);
	reg_wizchip_spi_cbfunc(W5X00spi_rb, W5X00spi_wb);
}

void setW5X00UART(UART_HandleTypeDef* newuart) {
	huart2 = newuart;
}

void W5X00_init(uint8_t ip[4]) {
	wizchip_init(bufSize, bufSize);
	wiz_NetInfo netInfo = { 	.mac 	= {0x01, 0x00, 0x00, 0x00, 0x00, 0x01},
								.ip 	= {ip[0], ip[1], ip[2], ip[3]},
								.sn 	= {255, 255, 255, 0},
								.gw		= {192, 168, 2, 1},
								.dhcp	= 1
	};
	wizchip_setnetinfo(&netInfo);
	PRINT_NETINFO(netInfo);
	wizchip_getnetinfo(&netInfo);
	PRINT_NETINFO(netInfo);
}

uint8_t W5X00_open_wait(uint32_t timeoutms) {
	uint32_t endtime = HAL_GetTick() + timeoutms;

	if((retVal = socket(0, Sn_MR_TCP, 5000, 0)) == 0) {
		/* Put socket in LISTEN mode. This means we are creating a TCP server */
		if((retVal = listen(0)) == SOCK_OK) {
			/* While socket is in LISTEN mode we wait for a remote connection */
			while((sockStatus = getSn_SR(0)) == SOCK_LISTEN) {
				if(HAL_GetTick() > endtime) {
					disconnect(0);
					close(0);
					return 1;
				}
				HAL_Delay(100);
			}
			/* OK. Got a remote peer. Let's send a message to it */
			while(1) {
				/* If connection is ESTABLISHED with remote peer */
				if((sockStatus = getSn_SR(0)) == SOCK_ESTABLISHED) {
					uint8_t remoteIP[4];
					uint16_t remotePort;
					/* Retrieving remote peer IP and port number */
					getsockopt(0, SO_DESTIP, remoteIP);
					getsockopt(0, SO_DESTPORT, (uint8_t*)&remotePort);
					sprintf(msg, CONN_ESTABLISHED_MSG, remoteIP[0], remoteIP[1], remoteIP[2], remoteIP[3], remotePort);
					PRINT_STR(msg);
					/* Let's send a welcome message and closing socket */
					if((retVal = send(0, GREETING_MSG, strlen(GREETING_MSG))) == (int16_t)strlen(GREETING_MSG))	PRINT_STR(SENT_MESSAGE_MSG);
					else { /* Ops: something went wrong during data transfer */
						sprintf(msg, WRONG_RETVAL_MSG, retVal);
						PRINT_STR(msg);
					}
					break;
				}
				else { /* Something went wrong with remote peer, maybe the connection was closed unexpectedly */
					sprintf(msg, WRONG_STATUS_MSG, sockStatus);
					PRINT_STR(msg);
					break;
				}
			}

		} else /* Ops: socket not in LISTEN mode. Something went wrong */
			PRINT_STR(LISTEN_ERR_MSG);
	} else { /* Can't open the socket. This means something is wrong with W5100 configuration: maybe SPI issue? */
		sprintf(msg, WRONG_RETVAL_MSG, retVal);
		PRINT_STR(msg);
	}

	/* We close the socket and start a connection again */
	disconnect(0);
	close(0);

	return 1;
}
