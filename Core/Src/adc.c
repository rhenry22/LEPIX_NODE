/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <stdint.h>

#define NUM_CHANNELS 2
#define NUM_CHANNELS2 1
#define MAX_SAMPLE_AGE 95
#define OVERSAMPLE  128

/* ADC1 Values (Normal Sensors) */
static volatile uint32_t num_samples = 0;
static volatile uint32_t last_sample = 0;
static volatile uint16_t samples[NUM_CHANNELS] = {0};
static volatile uint32_t samples_avg[NUM_CHANNELS] = {0};

/* ADC2 Values (CCS2_CP Input) */
static volatile uint32_t num_samples2h = 0;
static volatile uint32_t num_samples2l = 0;
static volatile uint16_t samples2[NUM_CHANNELS2] = {0};
static volatile uint32_t samples2_h[NUM_CHANNELS2] = {0};
static volatile uint32_t samples2_l[NUM_CHANNELS2] = {0};

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)samples, NUM_CHANNELS);

  /* USER CODE END ADC1_Init 2 */

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA4     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = EVSE_PP_Pin|ADC1_VAC_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PA2     ------> ADC2_IN2
    */
    GPIO_InitStruct.Pin = CCS2_CP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(CCS2_CP_GPIO_Port, &GPIO_InitStruct);

    /* ADC2 DMA Init */
    /* ADC2 Init */
    hdma_adc2.Instance = DMA2_Stream3;
    hdma_adc2.Init.Channel = DMA_CHANNEL_1;
    hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc2.Init.Mode = DMA_NORMAL;
    hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc2);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)samples2, NUM_CHANNELS2);

  /* USER CODE END ADC2_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0-WKUP     ------> ADC1_IN0
    PA4     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOA, EVSE_PP_Pin|ADC1_VAC_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE END ADC1:ADC_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PA2     ------> ADC2_IN2
    */
    HAL_GPIO_DeInit(CCS2_CP_GPIO_Port, CCS2_CP_Pin);

    /* ADC2 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    HAL_NVIC_DisableIRQ(ADC_IRQn);
  /* USER CODE END ADC2:ADC_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  adcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* adcHandle)
{
  uint8_t i;

  if (adcHandle == &hadc1)
  {
    last_sample = HAL_GetTick();

    for (i=0; i<NUM_CHANNELS; ++i)
      samples_avg[i] += samples[i];
    num_samples++;

    if (num_samples < OVERSAMPLE)
    {
      /* Keep sampling */
      HAL_ADC_Start_DMA(&hadc1, (uint32_t*)samples, NUM_CHANNELS);
    }
  }
  else if (adcHandle == &hadc2)
  {
    for (i=0; i<NUM_CHANNELS2; ++i)
    {
      if (samples2[i] <= 1613)
      {
        samples2_l[i] += samples2[i];
        num_samples2l++;
      }
      else
      {
        samples2_h[i] += samples2[i];
        num_samples2h++;
      }

    }

    /* Keep sampling */
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)samples2, NUM_CHANNELS2);
  }
}

/**
  * @brief  Get value of specified ADC channel
  * @param  channel Channel to read
  * @param  val Pointer to read value
  * @retval HAL_StatusTypeDef HAL_OK on success
  */
HAL_StatusTypeDef MX_ADC1_Get_Sample(uint8_t channel, uint16_t *val)
{
  uint32_t timeout;

  if (channel >= NUM_CHANNELS)
    return HAL_ERROR;

  if (HAL_GetTick() > (last_sample + MAX_SAMPLE_AGE))
  {
    HAL_StatusTypeDef ret;
    ret = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)samples, NUM_CHANNELS);
    if(ret != HAL_OK)
    {
      /* Start Conversion Error */
      return ret;
    }

    /* Wait for Conversion / Timeout */
    timeout = HAL_GetTick() + MAX_SAMPLE_AGE;
    while (HAL_GetTick() > (last_sample + MAX_SAMPLE_AGE))
    {
      if (HAL_GetTick() >= timeout)
      {
        return HAL_TIMEOUT;
      }
    }
  }

  if (val)
    *val = samples[channel];

  return HAL_OK;
}

/**
  * @brief  Get averaged value of specified ADC channel
  * @param  channel Channel to read
  * @param  val Pointer to read value
  * @retval HAL_StatusTypeDef HAL_OK on success
  */
HAL_StatusTypeDef MX_ADC1_Get_Sample_Avg(uint8_t channel, uint16_t *val)
{
  uint8_t i;

  if (channel >= NUM_CHANNELS)
    return HAL_ERROR;

  num_samples = 0;
  for (i=0; i<NUM_CHANNELS; ++i)
    samples_avg[i] = 0;

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)samples, NUM_CHANNELS);

  while (num_samples < OVERSAMPLE);

  *val = samples_avg[channel] / num_samples;

  return HAL_OK;
}


/**
  * @brief  Get averaged value of specified ADC channel
  * @param  channel Channel to read
  * @param  val Pointer to read value
  * @retval HAL_StatusTypeDef HAL_OK on success
  */
HAL_StatusTypeDef MX_ADC2_Get_Sample_Avgs(uint8_t channel, uint16_t *val_high, uint16_t *val_low)
{
  uint8_t i;
  HAL_StatusTypeDef ret = HAL_OK;

  if (channel >= NUM_CHANNELS2)
    return HAL_ERROR;

  if (num_samples2h > 0)
    *val_high = samples2_h[channel] / num_samples2h;
  if (num_samples2l > 0)
    *val_low = samples2_l[channel] / num_samples2l;

  if (num_samples2l == 0)
    *val_low = *val_high;
  else if (num_samples2h == 0)
    *val_high = *val_low;
  else if (num_samples2h == 0 && num_samples2l == 0)
    ret = HAL_ERROR;

  for (i=0; i<NUM_CHANNELS2; ++i)
  {
    samples2_h[i] = 0;
    samples2_l[i] = 0;
  }
  num_samples2h = 0;
  num_samples2l = 0;

  return ret;
}

/* USER CODE END 1 */
