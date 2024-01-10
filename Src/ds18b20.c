#include "ds18b20.h"

__STATIC_INLINE void delay_micro(__IO uint32_t micros){
    micros *= (72000000 / 1000000) / 9;
    while (micros--);
}

void ds18b20_reset(DS18B20 *ds18b20){
    static uint8_t reset_cmd = 0xF0;

    ds18b20->OW_data_bus->BRR = 0x00000EA6;

    HAL_UART_Transmit(ds18b20->huart, &reset_cmd, 1, 0x1);

    ds18b20->OW_data_bus->BRR = 0x00000138;
}

void ds18b20_transmit(DS18B20 *ds18b20, uint8_t byte){
    static uint8_t true_bit = 0xFF;
    static uint8_t false_bit = 0x00;

    for(int i = 0; i < 8; i++){
        HAL_UART_Transmit(ds18b20->huart, (byte >> i & 1) ? &true_bit : &false_bit, 1, 0x1);
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