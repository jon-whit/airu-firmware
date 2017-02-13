//*****************************************************************************
// rom_pinmux.c
//
// configure the device pins for different signals
//
// Copyright (c) 2016, Texas Instruments Incorporated - http://www.ti.com/ 
// All rights reserved.
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 2/13/2017 at 10:29:40 AM
// by TI PinMux version 4.0.1482 
//
//*****************************************************************************

#include "pinmux.h" 
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
//*****************************************************************************
void PinMuxConfig(void)
{


    //
    // Set unused pins to PIN_MODE_0 with the exception of JTAG pins 16,17,19,20
    //
    MAP_PinModeSet(PIN_21, PIN_MODE_0);
    
    //
    // Enable Peripheral Clocks 
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA2, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_SDHOST, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralClkEnable(PRCM_I2CA0, PRCM_RUN_MODE_CLK);
    //
    // Configure PIN_50 for GPIO Input
    //
    MAP_PinTypeGPIO(PIN_50, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x1, GPIO_DIR_MODE_IN);
    //
    // Configure PIN_58 for GPIO Input
    //
    MAP_PinTypeGPIO(PIN_58, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_IN);
    //
    // Configure PIN_59 for GPIO Input
    //
    MAP_PinTypeGPIO(PIN_59, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_IN);
    //
    // Configure PIN_60 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_60, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x20, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_61 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_61, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x40, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_62 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_62, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA0_BASE, 0x80, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_63 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_63, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x1, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_64 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_64, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x2, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_01 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_15 for GPIO Output
    //
    MAP_PinTypeGPIO(PIN_15, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA2_BASE, 0x40, GPIO_DIR_MODE_OUT);
    //
    // Configure PIN_18 for GPIO Input
    //
    MAP_PinTypeGPIO(PIN_18, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_IN);
    //
    // Configure PIN_53 for GPIO Input
    //
    MAP_PinTypeGPIO(PIN_53, PIN_MODE_0, false);
    MAP_GPIODirModeSet(GPIOA3_BASE, 0x40, GPIO_DIR_MODE_IN);
    //
    // Configure PIN_06 for SDHost0 SDCARD_DATA
    //
    MAP_PinTypeSDHost(PIN_06, PIN_MODE_8);
    //
    // Configure PIN_07 for SDHost0 SDCARD_CLK
    //
    MAP_PinTypeSDHost(PIN_07, PIN_MODE_8);
    //
    // Configure PIN_08 for SDHost0 SDCARD_CMD
    //
    MAP_PinTypeSDHost(PIN_08, PIN_MODE_8);
    //
    // Configure PIN_03 for UART0 UART0_TX
    //
    MAP_PinTypeUART(PIN_03, PIN_MODE_7);
    //
    // Configure PIN_04 for UART0 UART0_RX
    //
    MAP_PinTypeUART(PIN_04, PIN_MODE_7);
    //
    // Configure PIN_55 for UART1 UART1_TX
    //
    MAP_PinTypeUART(PIN_55, PIN_MODE_6);
    //
    // Configure PIN_57 for UART1 UART1_RX
    //
    MAP_PinTypeUART(PIN_57, PIN_MODE_6);
    //
    // Configure PIN_05 for I2C0 I2C_SCL
    //
    MAP_PinTypeI2C(PIN_05, PIN_MODE_5);
    //
    // Configure PIN_02 for I2C0 I2C_SDA
    //
    MAP_PinTypeI2C(PIN_02, PIN_MODE_1);
}