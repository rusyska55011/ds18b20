#include "main.h"
#include "stdio.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);

void USART2_IRQHandler(void);

enum bool_states {
    false = 0,
    true = 1
};

typedef struct DS18B20 {
    USART_TypeDef *OW_data_bus;
    _Bool is_on_a_bus;
    _Bool is_rx_complete;
    uint8_t row_data_bytes[8];
    float temp;
} DS18B20;

__STATIC_INLINE void delay_micro(__IO uint32_t micros){
    micros *= (72000000 / 1000000) / 9;
    while (micros--);
}

void ds18b20_reset(DS18B20 *ds18b20){
    static uint8_t reset_cmd = 0xF0;

    ds18b20->OW_data_bus->BRR = 0x00000EA6;

    HAL_UART_Transmit(&huart2, &reset_cmd, 1, 0x1);

    ds18b20->OW_data_bus->BRR = 0x00000138;
}

void ds18b20_transmit(DS18B20 *ds18b20, uint8_t byte){
    static uint8_t true_bit = 0xFF;
    static uint8_t false_bit = 0x00;

    for(int i = 0; i < 8; i++){
        HAL_UART_Transmit(&huart2, (byte >> i & 1) ? &true_bit : &false_bit, 1, 0x1);
    }
}

void ds18b20_set_settings(DS18B20 *ds18b20){
    ds18b20_reset(ds18b20);

    ds18b20_transmit(ds18b20, 0xCC);
    ds18b20_transmit(ds18b20, 0x4E); // Команда записи регистров

    ds18b20_transmit(ds18b20, 0x64); // 100 градусов
    ds18b20_transmit(ds18b20, 0x9E); // -30 градусов
    ds18b20_transmit(ds18b20, 0x7F); // Максимальная точность (Счет происходит за 700 млсек)
}

void ds18b20_start_measuring_cmd(DS18B20 *ds18b20){
    ds18b20_reset(ds18b20);
    ds18b20_transmit(ds18b20, 0xCC);
    ds18b20_transmit(ds18b20, 0x44);
}

void ds18b20_read_data_cmd(DS18B20 *ds18b20){
    ds18b20_reset(ds18b20);
    ds18b20_transmit(ds18b20, 0xCC);
    ds18b20_transmit(ds18b20, 0xBE);

//    uint64_t* row_data_bytes_ptr = (uint64_t*)ds18b20->row_data_bytes;
//    *(row_data_bytes_ptr) = 0;

    for (uint8_t i = 0; i < 8; i++)
        ds18b20->row_data_bytes[i] = 0;

    ds18b20->is_rx_complete = false;

    ds18b20->OW_data_bus->DR;
    ds18b20->OW_data_bus->CR1 |= USART_CR1_RXNEIE;
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    for(int i=0;i<8;i++){
        for(int j = 0; j < 8; j++){
            while(READ_BIT(ds18b20->OW_data_bus->SR, USART_SR_TXE) == 0);
            while((ds18b20->OW_data_bus->SR & USART_SR_RXNE) != RESET);
            ds18b20->OW_data_bus->DR = 0xFF;
        }
    }

    HAL_NVIC_DisableIRQ(USART2_IRQn);
    CLEAR_BIT(USART2->CR1, USART_CR1_RXNEIE);
}

void ds18b20_convert_row_data(DS18B20 *ds18b20){
    uint16_t row_temp = (ds18b20->row_data_bytes[1] << 8) | ds18b20->row_data_bytes[0];

    if ((row_temp >> 15) & 1){
        row_temp = ~row_temp;
        ds18b20->temp = (float)(row_temp / 16.0f) * -1;
    } else {
        ds18b20->temp = (float)(row_temp / 16.0f);
    }
}

DS18B20 my_ds18b20 = {
        .OW_data_bus = USART2,
        .is_on_a_bus = false,
        .is_rx_complete = true,
        .row_data_bytes = 0,
        .temp = 0
};

int main(void){
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USART1_UART_Init();

    ds18b20_set_settings(&my_ds18b20);

    while (1) {
        ds18b20_start_measuring_cmd(&my_ds18b20);

        HAL_Delay(800);

        ds18b20_read_data_cmd(&my_ds18b20);

        ds18b20_convert_row_data(&my_ds18b20);

        HAL_Delay(300);
    }
}

void USART2_IRQHandler(void){
    static uint8_t row_data_byte = 0;
    static uint8_t row_data_byte_bit = 0;

    static DS18B20 *my_ds18b20_ptr = &my_ds18b20;
    if (READ_BIT(my_ds18b20_ptr->OW_data_bus->SR, USART_SR_RXNE) == (USART_SR_RXNE) && my_ds18b20_ptr->is_rx_complete == false){
        if (row_data_byte_bit >= 8) {row_data_byte++; row_data_byte_bit = 0;}

        my_ds18b20_ptr->row_data_bytes[row_data_byte] |= (my_ds18b20_ptr->OW_data_bus->DR == 0xFF) ? 0b1 << row_data_byte_bit++ : 0b0 << row_data_byte_bit++;

        if (row_data_byte == 7 && row_data_byte_bit == 6) {my_ds18b20_ptr->is_rx_complete == true; row_data_byte = 0; row_data_byte_bit = 0;}
    }
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
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
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
    GPIOA->CRL &= ~(0xFF << 8U);
    GPIOA->CRL |= 0b0111 << 8U; // TX - OUT 50 Mhz, open drain
    GPIOA->CRL |= 0b0100 << 12U; // RX - IN, floating
    
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

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
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */

    /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
