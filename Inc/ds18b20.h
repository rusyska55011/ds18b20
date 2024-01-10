#ifndef WORK_DS18B20_H
#define WORK_DS18B20_H

#endif WORK_DS18B20_H

#include "stm32f1xx_hal.h"

enum bool_states {
    false = 0,
    true = 1
};

typedef struct DS18B20 {
    USART_TypeDef *OW_data_bus;
    UART_HandleTypeDef *huart;
    _Bool is_on_a_bus;
    _Bool is_rx_complete;
    uint8_t row_data_bytes[8];
    float temp;
} DS18B20;


__STATIC_INLINE void delay_micro(__IO uint32_t micros);

void ds18b20_reset(DS18B20 *ds18b20);

void ds18b20_transmit(DS18B20 *ds18b20, uint8_t byte);

void ds18b20_set_settings(DS18B20 *ds18b20);

void ds18b20_start_measuring_cmd(DS18B20 *ds18b20);

void ds18b20_read_data_cmd(DS18B20 *ds18b20);

void ds18b20_convert_row_data(DS18B20 *ds18b20);