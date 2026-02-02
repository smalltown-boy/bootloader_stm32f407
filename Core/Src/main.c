/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "W5500/w5500.h"																		*/
#include "socket.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define APP_ADDRESS 		0x08008000
#define APP_FLASH_START  	0x08008000
#define APP_FLASH_END    	0x08080000
#define START_COMMAND 		0xAA

#define WAIT_SYNC_BYTE 		0
#define PARS_PACKET 		1
#define END_SESSION 		2
#define ERASE_FLASH 		3

#define BOOTLOADER_SOCKET 	0
#define BOOTLOADER_PORT 	5555
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
wiz_NetInfo gSetNetInfo = {
	    .ip   = {10, 0, 28, 60},
	    .sn   = {255, 255, 0, 0},
	    .dhcp = NETINFO_STATIC };

uint8_t buffer_size_tx_rx[16] 			= {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};

uint8_t 	dest_ip[4];
uint16_t 	dest_port;

static uint32_t flash_write_addr 		= APP_FLASH_START;
static uint32_t total_received 			= 0;
static uint8_t  expected_ctrl   		= 0;
static bool     update_active   		= false;

int32_t ret_boot						= 0;
int32_t rx_boot							= 0;
uint8_t rx_data[1064];
uint8_t tx_data[16];
uint8_t first_byte						= 0;
bool	erase_result;

typedef enum
{
    PKT_OK,
    PKT_BAD_FORMAT,
    PKT_FLASH_ERROR,
    PKT_END_CRC
} packet_result_t;

typedef enum
{
	STATE_WAIT_SYNC_BYTE,
	STATE_PREPARE_FLASH,
	STATE_WAIT_DATA_PACKET,
	STATE_END_SESSION,
	STATE_ERASE_FLASH
} state_machine;

uint8_t  		ack						= 0;
uint16_t 		crc;
packet_result_t res;

state_machine 	state;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
typedef void 	(*pFunction)				(void);
void 			jump_to_application			(void);

void 			cris_enter					(void);
void 			cris_exit					(void);
void 			cs_select					(void);
void 			cs_deselect					(void);
uint8_t 		spi_readbyte				(void);
void 			spi_writebyte				(uint8_t wb);
void 			spi_readburst				(uint8_t* pBuf, uint16_t len);
void 			spi_writeburst				(uint8_t* pBuf, uint16_t len);

bool 			flash_erase					(void);
bool 			flash_write_block			(uint8_t *data, uint32_t len);
uint16_t 		crc16						(const uint8_t *data, uint32_t len);
bool 			verify_flash_crc			(uint16_t expected_crc);


packet_result_t bootloader_process_packet	(uint8_t *buf, uint32_t len, uint8_t *ack_ctrl, uint16_t *final_crc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);															/* Задержка после инициализации интерфейсов */
  if((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_SET)) {				/* Проверка на джампер. Если он установлен, переход на основную прошивку запрещён */
	  jump_to_application();												/* Переход к основному приложению */
  }

  reg_wizchip_cs_cbfunc(cs_select, cs_deselect);							/* Регистрация функций SPI для сетевого чипа W5500 */
  reg_wizchip_spi_cbfunc(spi_readbyte, spi_writebyte);
  reg_wizchip_cris_cbfunc(cris_enter, cris_exit);
  reg_wizchip_spiburst_cbfunc(spi_readburst, spi_writeburst);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);						/* Физический сброс W5500 */
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_Delay(500);

  ctlnetwork(CN_SET_NETINFO, (void*)&gSetNetInfo);							/* Инициализация сетевого чипа параметрами (ip, mask, etc. */
  ctlwizchip(CW_INIT_WIZCHIP,(void*)buffer_size_tx_rx);						/* Установка размеров буфера */

  socket(BOOTLOADER_SOCKET, Sn_MR_UDP, BOOTLOADER_PORT, 0);					/* Открытие сокета */

  state = STATE_WAIT_SYNC_BYTE;												/* Переходим в стартовое состояние машины состояний */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  switch(state)
	  {
	  	  case STATE_WAIT_SYNC_BYTE:										/* Ожидаем байт синхронизации */
	  		  ret_boot = getSn_RX_RSR(0);									/* Проверяем сокет на наличие данных */
	  		  if(ret_boot > 0)	{											/* Если данные имеются */
	  			  rx_boot = recvfrom(0, rx_data, 1, dest_ip, &dest_port);	/* Начинаем их принимать */
	  			  if(rx_boot > 0)	{										/* Если количество байт больше 0 (т.е. в сокете что-то есть) */
	  				  first_byte = rx_data[0];								/* Смотрим принятый от внешнего ПО байт */
	  				  if(first_byte == 0xAA)	{							/* Если первый принятый байт равен 0xAA */
	  					  state = STATE_PREPARE_FLASH;						/* В случае успеха переходим к подготовке flash памяти */
	  				  }	else {
	  					  state = STATE_WAIT_SYNC_BYTE;						/* В случае неудачи продролжаем ожидать правильный байт */
	  				  }
	  				  memset(rx_data, 0, sizeof(rx_data));					/* Очищаем буфер после обработки принятых данных */
	  			  }
	  		  }
	  		  break;

	  	  case STATE_PREPARE_FLASH:											/* Подготавливаем flash память к записи прошивки, т.к. это происходит "на лету" */
	  		  erase_result = flash_erase();									/* Стираем flash память */
	  		  if(erase_result == true)	{									/* Если стирание flash пасять прошло успешно */
	  			  tx_data[0] = 0xBB;										/* Отслыаем во внешнее ПО байт 0xBB - готовность к принятию прошивки */
	  			  sendto(0, tx_data, 1, &dest_ip, dest_port);
	  			  state = STATE_WAIT_DATA_PACKET;							/* Начинаем ожидать посылку с прошивкой */
	  		  } else {
	  			  tx_data[0] = 0xCC;										/* В случае ошибки отсылаем байт ошибки - 0xCC */
	  			  sendto(0, tx_data, 1, &dest_ip, dest_port);
	  			  state = STATE_WAIT_SYNC_BYTE;								/* Переходим в режим ожидания синхробайта */
	  		  }
	  		  memset(rx_data, 0, sizeof(rx_data));							/* Очищаем буфер после обработки принятых данных */
	  		  break;

	  	  case STATE_WAIT_DATA_PACKET:													/* Ожидаем пакеты с прошивкой */
	  		ret_boot = getSn_RX_RSR(0);													/* Проверяем сокет на наличие данных */
	  		if(ret_boot > 0) {															/* Если данные имеются */
	  			rx_boot = recvfrom(0, rx_data, sizeof(rx_data), dest_ip, &dest_port);	/* Начинаем их принимать */
	  			if(rx_boot > 0) {														/* Если количество байт больше 0 (т.е. в сокете что-то есть) */
	  				res = bootloader_process_packet(rx_data, rx_boot, &ack, &crc);		/* Сразу передаём полученные данные в функцию записи flash */

	  				switch(res)															/* Машина состояний, отслеживающая процесс записи во flash */
	  				{
	  					case PKT_OK:													/* Если пакет был записан без ошибок */
	  						sendto(0, &ack, 1, &dest_ip, dest_port);					/* Отправляем в ПО код готовности принятия следующего пакета */
	  						break;

	  					case PKT_FLASH_ERROR:											/* Если произошла ошибка записи */
	  						tx_data[0] = 0xCC;											/* Сообщим об этом внешнему ПО */
	  						sendto(0, tx_data, 1, &dest_ip, dest_port);
	  						state = STATE_WAIT_SYNC_BYTE;								/* Переходим к ожиданию синхробайта (начало приёма и записи прошивки сначала */
	  						break;

	  					case PKT_END_CRC:												/* Если был принят последний пакет с контрольной суммой */
	  						if(verify_flash_crc(crc)) {									/* Рассчёт контрольной суммы */
	  							tx_data[0] = 0xE2;										/* Если она правильная, сообщим об этом внешнему ПО и завершим приём данных */
	  							sendto(0, tx_data, 1, &dest_ip, dest_port);
	  							state = STATE_END_SESSION;
	  						} else {													/* Если контрольная сумма неправильная */
	  							tx_data[0] = 0xE1;										/* Сообщим об этом внешнему ПО */
	  							sendto(0, tx_data, 1, &dest_ip, dest_port);
	  							state = STATE_WAIT_SYNC_BYTE;							/* Переход в начало процедуры (ожидание синхробайта) */
	  						}
	  						break;

	  					default:														/* При неизвестной ошибке отправим во внешнее ПО код 0xEE */
	  						tx_data[0] = 0xEE;
	  						sendto(0, tx_data, 1, &dest_ip, dest_port);
	  						state = STATE_WAIT_SYNC_BYTE;								/* Переход в начало процедуры (ожидание синхробайта) */
	  						break;
	  				}
	  				memset(rx_data, 0, sizeof(rx_data));								/* Очищаем буфер после обработки принятых данных */
	  			}
	  		}
	  		break;

	  	  case STATE_END_SESSION:														/* Состояние завершения приёма пакетов */
	  		  HAL_SPI_DeInit(&hspi2);													/* Деинициализация интерфейса SPI */
	  		  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);										/* Деиницализация джампера, вывода сброса и chip select сетевого чипа */
	  		  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
	  		  HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0);
	  		  jump_to_application();													/* Переход в основную программу */
	  		  break;
	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(W5500_RESET_GPIO_Port, W5500_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BOOT_JAMPER_Pin */
  GPIO_InitStruct.Pin = BOOT_JAMPER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOOT_JAMPER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_RST_Pin */
  GPIO_InitStruct.Pin = SPI2_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W5500_RESET_Pin */
  GPIO_InitStruct.Pin = W5500_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W5500_RESET_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void jump_to_application(void) {
    uint32_t appStack = *(uint32_t*)APP_ADDRESS;
    uint32_t appEntry = *(uint32_t*)(APP_ADDRESS + 4);

    if (appStack < 0x20000000 || appStack > 0x20020000)
        return;

    __disable_irq();

    HAL_DeInit();
    SysTick->CTRL = 0;

    SCB->VTOR = APP_ADDRESS;

    __set_MSP(appStack);

    pFunction resetHandler = (pFunction)appEntry;
    resetHandler();
}

void 	cs_select(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, RESET);
}

void 	cs_deselect(void) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
}

void 	cris_enter(void) {
	__set_PRIMASK(1);
}

void 	cris_exit(void) {
	__set_PRIMASK(0);
}

uint8_t spi_readbyte(void) {
	uint8_t data;
	HAL_SPI_Receive(&hspi2,&data,1,100);
	return data;
}

void 	spi_writebyte(uint8_t wb) {
	HAL_SPI_Transmit(&hspi2,&wb,1,100);
}

void 	spi_readburst(uint8_t* pBuf, uint16_t len) {
	HAL_SPI_Receive(&hspi2, pBuf, len, HAL_MAX_DELAY);
}

void 	spi_writeburst(uint8_t* pBuf, uint16_t len) {
	HAL_SPI_Transmit(&hspi2, pBuf, len, HAL_MAX_DELAY);
}

bool flash_erase(void) {
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error;

    HAL_FLASH_Unlock();

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    erase.Sector = FLASH_SECTOR_2;					/* Первый сектор основного приложения */
    erase.NbSectors = 6;

    if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    HAL_FLASH_Lock();
    return true;
}

bool flash_write_block(uint8_t *data, uint32_t len) {
    uint32_t i;
    uint32_t word;

    if ((len % 4) != 0)
        return false;

    HAL_FLASH_Unlock();

    for (i = 0; i < len; i += 4) {
        word =
            (data[i + 0] << 0)  |
            (data[i + 1] << 8)  |
            (data[i + 2] << 16) |
            (data[i + 3] << 24);

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_write_addr, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }

        flash_write_addr += 4;
        total_received += 4;
    }

    HAL_FLASH_Lock();
    return true;
}

packet_result_t bootloader_process_packet(uint8_t *buf, uint32_t len, uint8_t *ack_ctrl, uint16_t *final_crc) {
    uint16_t data_len;

    // Минимальная длина
    if (len < 4)
        return PKT_BAD_FORMAT;

    // CMD
    if (buf[0] != 0x01)
        return PKT_BAD_FORMAT;

    // Контрольный байт — всегда возвращаем ПК
    *ack_ctrl = buf[1];

    /* ---------- CRC пакет ---------- */
    if (buf[1] == 0xEE)
    {
        if (len != 4)
            return PKT_BAD_FORMAT;

        *final_crc = (buf[2] << 8) | buf[3];
        return PKT_END_CRC;
    }

    /* ---------- DATA пакет ---------- */
    data_len = (buf[2] << 8) | buf[3];

    if (data_len == 0 || data_len > 1024)
        return PKT_BAD_FORMAT;

    if (len != (uint32_t)(4 + data_len))
        return PKT_BAD_FORMAT;

    // Данные должны быть кратны 4 байтам
    if (data_len % 4)
        return PKT_BAD_FORMAT;

    // Пишем во Flash
    if (!flash_write_block(&buf[4], data_len))
        return PKT_FLASH_ERROR;

    return PKT_OK;
}

uint16_t crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0xFFFF;
    uint8_t  bit;

    while (len--)
    {
        crc ^= (*data++) << 8;

        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }

    return crc;
}

bool verify_flash_crc(uint16_t expected_crc) {
    uint8_t  *flash_ptr;
    uint16_t calc_crc;

    if (total_received == 0)
        return false;

    // Защита от выхода за пределы Flash
    if (APP_FLASH_START + total_received > FLASH_END)
        return false;

    flash_ptr = (uint8_t *)APP_FLASH_START;

    calc_crc = crc16(flash_ptr, total_received);

    return (calc_crc == expected_crc);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
