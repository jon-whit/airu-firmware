/*
 * pms3003.h
 *
 *  Created on: Jan 31, 2017
 *      Author: Tom
 */

#ifndef PMS3003_H_
#define PMS3003_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define UART1_BAUD_RATE 9600
#define SYSCLK          80000000
#define PMS             UARTA1_BASE
#define PMS_PERIPH      PRCM_UARTA1

extern void InitPMS(void);
extern unsigned char* FillBuff(unsigned char *buf);
extern int CheckSum(unsigned char *buF);
unsigned int GetPM01(unsigned char *buf);
unsigned int GetPM2_5(unsigned char *buf);
unsigned int GetPM10(unsigned char *buf);

#ifdef __cplusplus
}
#endif

#endif /* PMS3003_H_ */
