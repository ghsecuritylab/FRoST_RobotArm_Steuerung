/**
  *****************************************************************************
  * @title  udp_lwip_MTR.h
  * @author Max Triller
  * @date   26.03.2019
  * @brief  Initialization of the UDP client, based on the LwIP stack for FreeRTOS
  * 		Allows you to send UDP messages to server with known IP. Client IP is set in "lwip.c".
  * 		Network monitor Software:
  * 		 	- use https://github.com/PavelBansky/EchoTool
  * 		 	- ore https://www.wireshark.org/ (to start NPF driver in Windows use "sc start npf" in command line prompt)
  * @edit	Max Triller 25.06.2019
  *******************************************************************************
  */

#ifndef __udp_lwip_MTR_H
#define __udp_lwip_MTR_H

/* Private Includes ----------------------------------------------------------*/
#include "udp.h"
#include "lwip/udp.h"

/* Private define ------------------------------------------------------------*/
/*setup destination IP address e.g. ground stations ip*/
#define	DEST_IP_ADDR0  192
#define	DEST_IP_ADDR1  168
#define	DEST_IP_ADDR2  0
#define	DEST_IP_ADDR3  21
#define UDP_DEST_PORT  2019

/*setup server (own) ip addres does not work - to change ip see lwip.c ->lwip initialization  */
#define SERVER_IP_ADR0	192
#define SERVER_IP_ADR1	168
#define SERVER_IP_ADR2	0
#define SERVER_IP_ADR3	25
#define UDP_SERVER_PORT 2019

/*Define time to wait for the Ethernet mutex to become free*/
#define UDP_LWIP_TIMEOUT_ETHERNET_MUTEX 20 //in ms

/* Private typedefs ----------------------------------------------------------*/
typedef enum
{
	UDP_LWIP_SEND_ERROR,
	UDP_LWIP_SEND_OK
}updLwip_enumTypeDef_sendReturn;

/* Private function prototypes -----------------------------------------------*/

/*UDP ETHERNET initialization functions*/
void udp_lwip_initInterface(void);			/*Note: Must be called while program initialization!*/
void udp_lwip_initClient(void); 			/*Note: Not in use! */
void udp_lwip_initServer(void);
void udp_lwip_initTransmitMutex(void); 		/* Call this on initialization when MUTEXES are used*/ /*Added MTR 22.06.2019*/

/* UDP ETHERNET transmit functions */
void udp_lwip_clientSendMessage(uint32_t myData); //Send 32 bit of data
uint8_t udp_lwip_txUdpMessage_gs(uint8_t dataID); //Send structure from global data structure ->see also globalDataStructure.h/c

/* UDP ETHERNET receive callback functions to handle incoming data  */
void udp_lwip_clientCallback	(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port); /*Note: clientCallback is not in use anymore MTR 24.06.19*/
void udp_lwip_rxCallback_gs		(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port); /*Note: rxCallback is responsible to receive data check id and store it to the right global data structure */
void udp_lwip_serverCallback	(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port); /*Note: clientCallback is not in use anymore MTR 24.06.19*/

#endif // __udp_lwip_MTR_H
