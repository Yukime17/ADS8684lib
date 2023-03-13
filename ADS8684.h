/*
 * ADS8684.h
 *
 *  Created on: Feb 23, 2023
 *      Author: nnc_1
 */

#ifndef INC_ADS8684_H_
#define INC_ADS8684_H_

#include <stdlib.h>
#include "stm32f4xx_hal.h"

/* command */
#define NO_OP			0x0000
#define STDBY			0x8200
#define RST				0x8500
#define AUTO_RST		0xA000
#define MAN_CH_0		0xC000
#define MAN_CH_1		0xC400
#define MAN_CH_2		0xC800
#define MAN_CH_3		0xCC00
/* command */

/*program register address*/
#define AUTO_SEQ_EN		0x01
#define CH_PWRDN		0x02
#define FSEL			0x03
#define	RANGE_CH0		0x05
#define RANGE_CH1		0x06
#define RANGE_CH2		0x07
#define RANGE_CH3		0x08
/*program register address*/

/*other defines*/
#define ADS8684_RST		HAL_GPIO_WritePin(ADC_RST_GPIO_Port, ADC_RST_Pin, GPIO_PIN_SET);
/*other defines*/

/*function prototypes*/
void MX_SPI1_Init(void);
void ADS8684_writeREG(uint8_t address, uint8_t data);
void ADS8684_Init(void);
void ADS8684_reset(void);
void ADS8684_STDBY(void);
uint16_t* ADS8684_readALL(void);
uint16_t ADS8684_readCH(int ch);
/*function prototypes*/



#endif /* INC_ADS8684_H_ */
