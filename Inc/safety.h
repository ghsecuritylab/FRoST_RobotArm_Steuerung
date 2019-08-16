/*
  *****************************************************************************
  * @title  savety.h
  *
  * @author Carsten Mueller
  *
  * @date   Created on: 16.07.2019
  *
  * @brief  Add last Ethernet RX timeout functions for stopping the rover when connection is lost
  *
  * @edit	 Max Triller 		15.07.2019	MTR
  *******************************************************************************
  */

#ifndef SAFETY_H_
#define SAFETY_H_

#include "main.h"

#define SAFETY_TIMEOUT_MS  (500) //time to trigger timeout

/* set actual time to timestamp_last_rx */
void safety_setTimestamp_last_rx();

/* reset timestamp_last_rx to 0 */
void safety_resetTimestamp_last_rx();

/* get timestamp_last_rx */
uint32_t safety_getTimestamp_last_rx();

#endif /* SAFETY_H_ */
