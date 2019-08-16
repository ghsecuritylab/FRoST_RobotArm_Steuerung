/*
  *****************************************************************************
  * @title   savety.c
  *
  * @author Carsten Mueller
  *
  * @date   Created on: 16.07.2019
  *
  * @brief  Add last Ethernet RX timeout functions for stopping the rover when connection is lost
  *
  * @edit	 Max Triller 		17.07.2019	MTR
  *******************************************************************************
  */

#include "safety.h"
#include "cmsis_os.h"


uint32_t timestamp_last_rx; //value will be set in Ethernet RX callback aka "udp_lwip_rxCallback_gs"

void safety_setTimestamp_last_rx()
{
    timestamp_last_rx = osKernelSysTick();
}

void safety_resetTimestamp_last_rx()
{
    timestamp_last_rx = 0;
}

uint32_t safety_getTimestamp_last_rx()
{
    return timestamp_last_rx;
}
