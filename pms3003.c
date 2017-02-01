/*
 * pms3003.c
 *
 *  Created on: Jan 31, 2017
 *      Author: Tom
 */

// Standard includes
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "prcm.h"
#include "pin.h"
#include "uart.h"
#include "rom.h"
#include "rom_map.h"

#if defined(USE_FREERTOS) || defined(USE_TI_RTOS)
#include "osi.h"
#endif

#include "pms3003.h"

#define UartGetChar()       MAP_UARTCharGet(PMS)
#define BUFFER_LENGTH       24
#define PM01_HIGH           4
#define PM01_LOW            5
#define PM2_5_HIGH          6
#define PM2_5_LOW           7
#define PM10_HIGH           8
#define PM10_LOW            9

//*****************************************************************************
//
//! Initialization
//!
//! This function
//!        1. Configures the UART to be used.
//!
//! \return none
//
//*****************************************************************************
void
InitPMS()
{
  MAP_UARTConfigSetExpClk(PMS,MAP_PRCMPeripheralClockGet(PMS_PERIPH),
                    UART1_BAUD_RATE, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                     UART_CONFIG_PAR_NONE));
}


//*****************************************************************************
//
//! FillBuffer
//!
//!    @brief  Fill buffer with data from particulate matter sensor
//!
//!     @param  *buf is the buffer to fill
//!
//!     @return filled buffer
//!
//!
//
//*****************************************************************************
long*
FillBuffer(long *buf)
{

    // TODO: Wait state / error checking / RTOS functionality (osi sleep)
    int i;
    for(i=0;i<BUFFER_LENGTH;i++){
        if(!MAP_UARTBusy(PMS)){

            // UARTCharGet casts to long before returning
            buf[i] = UARTGetChar();
        }
    }

    return buf;
}


//*****************************************************************************
//
//! CheckSum
//!
//!    @brief  Returns the checksum from the buffer. The last 2 bytes should
//!             equal the other 22 bytes.
//!
//!     @param  *buf is the buffer from PMS3003 uart
//!
//!     @return True if checksum passed, False otherwise
//!
//!
//
//*****************************************************************************
tBoolean
CheckSum(long *buf)
{
    long checkSum = 0x0000;
    checkSum = (buf[BUFFER_LENGTH-2]<<8)|(buf[BUFFER_LENGTH-1] & 0xff);
    unsigned long sum = 0;
    int i;
    for(i=0;i<BUFFER_LENGTH-2;i++){
        sum += buf[i];
    }

    if (sum == checkSum){
        return 1;
    }
    else{
        return 0;
    }
}


//*****************************************************************************
//
//! GetPM01
//!
//!    @brief  Get PM 1um data from buffer
//!
//!     @param  *buf the PM buffer
//!
//!     @return PM 1um data
//!
//!
//
//*****************************************************************************
unsigned long
GetPM01(long *buf)
{
    unsigned long PM = 0x0000;
    PM = (buf[PM01_HIGH]<<8)|(buf[PM01_LOW] & 0xff);
    return PM;
}


//*****************************************************************************
//
//! GetPM01
//!
//!    @brief  Get PM 2.5um data from buffer
//!
//!     @param  *buf the PM buffer
//!
//!     @return PM 2.5um data
//!
//!
//
//*****************************************************************************
unsigned long
GetPM2_5(long *buf)
{
    unsigned long PM = 0x0000;
    PM = (buf[PM2_5_HIGH]<<8)|(buf[PM2_5_LOW] & 0xff);
    return PM;
}


//*****************************************************************************
//
//! GetPM01
//!
//!    @brief  Get PM 10um data from buffer
//!
//!     @param  *buf the PM buffer
//!
//!     @return PM 10um data
//!
//!
//
//*****************************************************************************
unsigned long
GetPM10(long *buf)
{
    unsigned long PM = 0x0000;
    PM = (buf[PM10_HIGH]<<8)|(buf[PM10_LOW] & 0xff);
    return PM;
}


