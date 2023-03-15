/*
 * ADS8684.c
 *
 *  Created on: Feb 24, 2023
 *      Author: nnc_1
 */

#include "ADS8684.h"

static SPI_HandleTypeDef hspi1;

void Error_Handler(void);
/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief ADS8684 write register function
  * @param address of the register, data to be sent
  * @retval None
  */
void ADS8684_writeREG(uint8_t address, uint8_t data){
	uint8_t reg_temp = 0x00;
	reg_temp = (address << 1) | 0x01;
	uint8_t CMD_tobesent[2] = {reg_temp, data};
	HAL_SPI_Transmit(&hspi1, (uint8_t *)CMD_tobesent, 2, 1000);
}

/**
  * @brief ADS8684 initialization
  * @param None
  * @retval None
  */
void ADS8684_Init(void){

	//set all channel power on
	ADS8684_writeREG(CH_PWRDN, 0x00);	//0 = on, 1 = off, each bit represents each channel

	//set feature select register
	ADS8684_writeREG(FSEL, 0x00);		//mainly for daisy-chain, refer to datasheet

	//set range for each channel
	/*	RANGE SELECTION
		0x00 = +-2.5V * Vref
		0x01 = +-1.25V * Vref
		0x05 = 2.5 * Vref
		0x06 = 1.25 * Vref
	 */
	ADS8684_writeREG(RANGE_CH0, 0x05);	//channel 0
	ADS8684_writeREG(RANGE_CH1, 0x05);	//channel 1
	ADS8684_writeREG(RANGE_CH2, 0x05);	//channel 2
	ADS8684_writeREG(RANGE_CH3, 0x05);	//channel 3

	//set auto scan for CH0-CH3
	ADS8684_writeREG(AUTO_SEQ_EN, 0x0F);

}

/**
  * @brief ADS8684 reset register function
  * @param None
  * @retval None
  */
void ADS8684_reset(void){
	ADS8684_RST;
}

/**
  * @brief ADS8684 standby function
  * @param None
  * @retval None
  */
void ADS8684_STDBY(void){
	uint8_t CMD_tobesent[2] = {(uint8_t) (STDBY >> 8), (uint8_t) STDBY};
	HAL_SPI_Transmit(&hspi1, (uint8_t *)CMD_tobesent, 2, 1000);
}

/**
  * @brief ADS8684 read and convert all channel function
  * @param None
  * @retval uint16_t array of size 4 containing the reading of each channel
  */
uint16_t* ADS8684_readALL(void){
	uint8_t temp;
	uint8_t AUTORESET[2] = {(uint8_t)(AUTO_RST >> 8), (uint8_t)AUTO_RST};
	uint8_t NOOP[2] = {(uint8_t)(NO_OP >> 8), (uint8_t)NO_OP};
	uint8_t data[2] = {0x00, 0x00};
	uint16_t* ret = malloc(sizeof(uint16_t) * 8);
	HAL_SPI_Transmit(&hspi1, (uint8_t *)AUTORESET, 2, 1000);
	for(int i = 0; i < 4; i++){
		HAL_SPI_Transmit(&hspi1, (uint8_t *)NOOP, 2, 1000);
		HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)NOOP, (uint8_t *)data, 2, 1000);
		temp = (data[0] << 8) | data[1];
		ret[i] = temp;
	}
	return ret;
}

/**
  * @brief ADS8684 read and convert 1 channel function
  * @param channel to be read (int)
  * @retval uint16_t containing the reading of each channel
  */
static uint16_t ADS8684_readCH(int ch){
	uint16_t ret;
	uint16_t cmd[4] = {MAN_CH_0, MAN_CH_1, MAN_CH_2, MAN_CH_3};
	uint8_t data[2] = {0x00, 0x00};
	uint8_t dummy[2] = {0x00, 0x00};
	uint8_t SEND[2] = {(uint8_t)(cmd[ch] >> 8), (uint8_t)(cmd[ch])};
	HAL_SPI_Transmit(&hspi1, (uint8_t *)SEND, 2, 1000);
	HAL_SPI_Transmit(&hspi1,  (uint8_t *)dummy, 2, 1000);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)dummy, (uint8_t *)data, 2, 1000);
	ret = (data[0] << 8) | data[1];
	return ret;
}
