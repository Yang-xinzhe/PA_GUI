/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "main.h"
/* USER CODE BEGIN 0 */
static uint8_t ram[256];
static uint8_t offset; // index of current RAM cell
static uint8_t first=1;// first byte --> new offset
int Temp_Max_threshold = 85;
int Temp_recover_threshold = 65;
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00101D2D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00101D2D;
  hi2c2.Init.OwnAddress1 = 106;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_DISABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOF_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PF6     ------> I2C2_SCL
    PF7     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PF6     ------> I2C2_SCL
    PF7     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_6);

    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_7);

    /* I2C2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C2_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* TEMP SENSOR DATA */
float Read_Temperature(uint8_t i2c_address)
{
	  uint8_t temp_data[2];
    int16_t temp;
    float temperature;
		HAL_Delay(100); 
    HAL_I2C_Master_Receive(&hi2c1, (i2c_address << 1) | 0x01, temp_data, 2, HAL_MAX_DELAY);
	
    temp = (temp_data[0] << 8) | temp_data[1];
    temp >>= 4;
    temperature = temp * 0.0625;
		if(temperature == 0){
			Debug_print("I2C Temperature is 0!!!!\r\n");
		}
    return temperature;
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  first = 1;
  HAL_I2C_EnableListen_IT(hi2c); // slave is ready again
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	ram[80] = Read_Temperature(0x49); // Test
	ram[81] = Read_Temperature(0x48);
  if( TransferDirection==I2C_DIRECTION_TRANSMIT ) {
    if( first ) {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, &offset, 1, I2C_NEXT_FRAME);
    } else {
      HAL_I2C_Slave_Seq_Receive_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
    }
  } else {
		if(offset == 0x7F) {																									// i2cget -fy 1 0x35 0x80 // Main amplifier temperature check
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &ram[80], 1, I2C_NEXT_FRAME);
		} else if(offset == 0x78) {																						// i2cget -fy 1 0x35 0x79 // Sub amplifier temperature check
			HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &ram[81], 1, I2C_NEXT_FRAME);
		}	
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if(first) {
    first = 0;
  } else {
//    offset++;
  }
	if (offset == 0x93 && ram[offset] == 0x00) {
		Tor_Switch(1); // Forwards
	} else if (offset == 0x93 && ram[offset] == 0x01) {
		Tor_Switch(0); // Backwards
	}	else if(offset == 0x90) {
		Temp_Max_threshold = ram[offset];
	}	else if(offset == 0x91) {
		Temp_recover_threshold = ram[offset];
	}
  HAL_I2C_Slave_Seq_Receive_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  offset++;
  HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &ram[offset], 1, I2C_NEXT_FRAME);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if( HAL_I2C_GetError(hi2c)==HAL_I2C_ERROR_AF ) {
    offset--; // transaction terminated by master
  } else {
  }
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
