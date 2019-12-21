/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
enum UartState
{
  CMD_INPUT,
  DELIM_INPUT,
  ARG_INPUT
};

enum Command
{
  LIGHT_CMD,
  BLOW_CMD,
  SAY_CMD,
  UNKNOWN_CMD
};

enum Led
{
  UNKNOWN_LED,
  RED_LED = LED_RED_Pin,
  YELLOW_LED = LED_YELLOW_Pin,
  GREEN_LED = LED_GREEN_Pin
};

enum DisplayLine
{
  UP,
  DOWN
};

/* Private define ------------------------------------------------------------*/
#define CMD_ARG_DELIM ' '
#define NEW_LINE "\r"

#define LIGHT_CMD_STR "LIGHT"
#define BLOW_CMD_STR "BLOW"
#define RED_LED_ARG_STR "RED"
#define YELLOW_LED_ARG_STR "YELLOW"
#define GREEN_LED_ARG_STR "GREEN"

#define SAY_CMD_STR "SAY"

#define ERROR_MSG "Error"
#define SUCCESS_MSG "OK"

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
char* command = NULL, * argument = NULL;
uint8_t uartInputBuf;
enum UartState state = CMD_INPUT;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);

bool ProcessCommand(void);
bool ProcessLedCmd(const bool light);
bool ProcessSayCmd(void);
void OutputCmdResult(const bool cmdResult);
uint8_t GetCommandChecksum(void);
enum Command ParseCmd(void);
enum Led ParseLedArg(void);

void UartRunReceiveInterrupt(void);
void PrintToUart(uint8_t* data, const uint16_t length);
void PrintLineToUart(uint8_t* data, const uint16_t length);
void ResetUart(void);

void InitDisplay(void);
void ResetDisplay(void);
void PrintNextSymbolToDisplay(const char symbol);
void PrintNextStringToDisplay(const char* str);
void SendToDisplay(const bool isSymbol, const uint8_t data, const uint32_t cmdDelay);
void SetDisplayAddress(const uint8_t address);
void SetDisplayLine(const enum DisplayLine line);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  
  ResetUart();
  InitDisplay();
  UartRunReceiveInterrupt();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart1)
  {
    switch (state)
    {
      case CMD_INPUT:
        {
          if (uartInputBuf == CMD_ARG_DELIM)
          {
            state = DELIM_INPUT;
          }
          else
          {
            const size_t newCommandLength = strlen(command) + 1;
            command = (char*)realloc(command, (newCommandLength + 1) * sizeof(char));
            command[newCommandLength - 1] = uartInputBuf;
            command[newCommandLength] = '\0';
            char* newLinePtr = strstr(command, NEW_LINE);
            if (newLinePtr == command)
            {
              ResetUart();
            }
            else if (newLinePtr != NULL)
            {
              *newLinePtr = '\0';
              OutputCmdResult(ProcessCommand());
              ResetUart();
            }
          }
        }
        break;
      case DELIM_INPUT:
        {
          if (uartInputBuf != CMD_ARG_DELIM)
          {
            argument = (char*)realloc(argument, 2 * sizeof(char));
            argument[0] = uartInputBuf;
            argument[1] = '\0';
            if (strcmp(argument, NEW_LINE) == 0)
            {
              argument[0] = '\0';
              OutputCmdResult(ProcessCommand());
              ResetUart();
            }
            else
            {
              state = ARG_INPUT;
            }
          }
        }
        break;
      case ARG_INPUT:
        {
          const size_t newArgumentLength = strlen(argument) + 1;
          argument = (char*)realloc(argument, (newArgumentLength + 1) * sizeof(char));
          argument[newArgumentLength - 1] = uartInputBuf;
          argument[newArgumentLength] = '\0';
          char* newLinePtr = strstr((char*)argument, NEW_LINE);
          if (newLinePtr != NULL)
          {
            *newLinePtr = '\0';
            OutputCmdResult(ProcessCommand());
            ResetUart();
          }
        }
        break;
    }
    UartRunReceiveInterrupt();
  }
}

void UartRunReceiveInterrupt(void)
{
  HAL_UART_Receive_IT(&huart1, &uartInputBuf, 1);
}

bool ProcessCommand(void)
{
  const enum Command cmd = ParseCmd();
  bool result;
  switch (cmd)
  {
    case SAY_CMD:
      result = ProcessSayCmd();
      break;
    case LIGHT_CMD:
      result = ProcessLedCmd(true);
      break;
    case BLOW_CMD:
      result = ProcessLedCmd(false);
      break;
    default:
      result = false;
      break;
  }
  return result;
}

void OutputCmdResult(const bool cmdResult)
{
  char* resultMessage;
  if (cmdResult)
  {
    const size_t strByteLength = 3;
    resultMessage = (char*)malloc((strlen(SUCCESS_MSG) + 1 + strByteLength + 1) * sizeof(char));
    sprintf(resultMessage, "%s %u", SUCCESS_MSG, GetCommandChecksum());
  }
  else
  {
    resultMessage = (char*)malloc((strlen(ERROR_MSG) + 1) * sizeof(char));
    strcpy(resultMessage, ERROR_MSG);
  }

  PrintLineToUart((uint8_t*)resultMessage, strlen(resultMessage) * sizeof(char));

  ResetDisplay();
  SetDisplayLine(UP);
  PrintNextStringToDisplay(command);
  PrintNextSymbolToDisplay(' ');
  PrintNextStringToDisplay(argument);
  SetDisplayLine(DOWN);
  PrintNextStringToDisplay(resultMessage);

  free(resultMessage);
}

uint8_t GetCommandChecksum(void)
{
  const size_t commandLength = strlen(command), argumentLength = strlen(argument);
  uint8_t result = 0;
  for (size_t i = 0; i < commandLength; ++i)
  {
    result ^= command[i];
  }
  for (size_t i = 0; i < argumentLength; ++i)
  {
    result ^= argument[i];
  }
  return result;
}

void PrintToUart(uint8_t* data, const uint16_t length)
{
  HAL_UART_Transmit(&huart1, data, length, INT32_MAX);
}

void PrintLineToUart(uint8_t* data, const uint16_t length)
{
  PrintToUart(data, length);
  PrintToUart((uint8_t*)NEW_LINE, strlen(NEW_LINE));
}

void ResetUart(void)
{
  command = (char*)realloc(command, 1 * sizeof(char));
  command[0] = '\0';
  argument = (char*)realloc(argument, 1 * sizeof(char));
  argument[0] = '\0';
  state = CMD_INPUT;
}

enum Command ParseCmd(void)
{
  enum Command result = UNKNOWN_CMD;
  if (strcmp(command, LIGHT_CMD_STR) == 0)
  {
    result = LIGHT_CMD;
  }
  else if (strcmp(command, BLOW_CMD_STR) == 0)
  {
    result = BLOW_CMD;
  }
  else if (strcmp(command, SAY_CMD_STR) == 0)
  {
    result = SAY_CMD;
  }
  return result;
}

enum Led ParseLedArg(void)
{
  enum Led result = UNKNOWN_LED;
  if (strcmp(argument, RED_LED_ARG_STR) == 0)
  {
    result = RED_LED;
  }
  else if (strcmp(argument, YELLOW_LED_ARG_STR) == 0)
  {
    result = YELLOW_LED;
  }
  else if (strcmp(argument, GREEN_LED_ARG_STR) == 0)
  {
    result = GREEN_LED;
  }
  return result;
}

bool ProcessLedCmd(const bool light)
{
  const enum Led led = ParseLedArg();
  const bool result = led != UNKNOWN_LED;
  if (led != UNKNOWN_LED)
  {
    HAL_GPIO_WritePin(GPIOA, (uint16_t)led, light ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
  return result;
}

bool ProcessSayCmd(void)
{
  PrintLineToUart((uint8_t*)argument, strlen(argument) * sizeof(char));
  return true;
}

void InitDisplay(void)
{
  HAL_Delay(16);
  const uint8_t setBusTo8BitAndDoubleLine = 0x3C,
    shiftOnWrite = 0x6,
    enableDisplay = 0xC;
  const uint8_t busCmdDelay = 1,
    shiftCmdDelay = 1,
    enableCmdDelay = 1;
  SendToDisplay(false, setBusTo8BitAndDoubleLine, busCmdDelay);
  SendToDisplay(false, shiftOnWrite, shiftCmdDelay);
  SendToDisplay(false, enableDisplay, enableCmdDelay);
  ResetDisplay();
}

void ResetDisplay(void)
{
  const uint8_t resetDataCmd = 0x1,
    resetDataDelay = 2;
  SendToDisplay(false, resetDataCmd, resetDataDelay);
}

void SendToDisplay(const bool isSymbol, const uint8_t data, const uint32_t cmdDelay)
{
  HAL_GPIO_WritePin(DISPLAY_CMD_GPIO_Port, DISPLAY_CMD_Pin, isSymbol ? GPIO_PIN_SET : GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOB, data, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, ~data, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(DISPLAY_CLK_GPIO_Port, DISPLAY_CLK_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(DISPLAY_CLK_GPIO_Port, DISPLAY_CLK_Pin, GPIO_PIN_RESET);
  HAL_Delay(cmdDelay);
  HAL_GPIO_WritePin(GPIOB, data, GPIO_PIN_RESET);
}

void PrintNextSymbolToDisplay(const char symbol)
{
  SendToDisplay(true, symbol, 1);
}

void PrintNextStringToDisplay(const char* str)
{
  const size_t printLength = strlen(str);
  for (size_t i = 0; i < printLength; ++i)
  {
    PrintNextSymbolToDisplay(str[i]);
  }
}

void SetDisplayAddress(const uint8_t address)
{
  SendToDisplay(false, 0x80 | (address & (~0x80)), 1);
}

void SetDisplayLine(const enum DisplayLine line)
{
  SetDisplayAddress(line == DOWN ? 0x40 : 0x0);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DISPLAY_DATA_0_Pin|DISPLAY_DATA_1_Pin|DISPLAY_DATA_2_Pin|DISPLAY_DATA_3_Pin 
                          |DISPLAY_DATA_4_Pin|DISPLAY_DATA_5_Pin|DISPLAY_DATA_6_Pin|DISPLAY_DATA_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|DISPLAY_CLK_Pin 
                          |DISPLAY_CMD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DISPLAY_DATA_0_Pin DISPLAY_DATA_1_Pin DISPLAY_DATA_2_Pin DISPLAY_DATA_3_Pin 
                           DISPLAY_DATA_4_Pin DISPLAY_DATA_5_Pin DISPLAY_DATA_6_Pin DISPLAY_DATA_7_Pin */
  GPIO_InitStruct.Pin = DISPLAY_DATA_0_Pin|DISPLAY_DATA_1_Pin|DISPLAY_DATA_2_Pin|DISPLAY_DATA_3_Pin 
                          |DISPLAY_DATA_4_Pin|DISPLAY_DATA_5_Pin|DISPLAY_DATA_6_Pin|DISPLAY_DATA_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RED_Pin LED_YELLOW_Pin LED_GREEN_Pin DISPLAY_CLK_Pin 
                           DISPLAY_CMD_Pin */
  GPIO_InitStruct.Pin = LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin|DISPLAY_CLK_Pin 
                          |DISPLAY_CMD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* User can add his own implementation to report the HAL error return state */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
