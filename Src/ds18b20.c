//#include "ds18b20.h"
//
//__STATIC_INLINE void delay_micro(__IO uint32_t micros){
//    micros *= (SystemCoreClock / 1000000) / 9;
//    while (micros--) ;
//}
//
//void ds18b20_port_init(void){
//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
//    GPIOB->CRH |= 0b0111 << 12U; // Инициализация B11
//}
//
//uint8_t ds18b20_reset(void){
//    uint16_t status;
//
//    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR11); // Низкий уровень
//    delay_micro(485); // Задержка минимум на 480 мс
//    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS11); // Высокий
//    delay_micro(65); // Задержка минимум на 60 мс
//    status = GPIOB->IDR & GPIO_IDR_IDR11; // Проверяем результат
//
//    delay_micro(500); // Задрежка минимум на 480 мс
//
//    return (status ? 1 : 0);
//}
//
//uint8_t ds18b20_read_bit(void){
//    uint8_t bit = 0;
//    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR11); // Низкий уровень
//    delay_micro(2);
//    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS11); // Высокий уровень
//    delay_micro(13);
//    bit = (GPIOB->IDR & GPIO_IDR_IDR11 ? 1 : 0); // Проверяем пришедший бит
//    delay_micro(45);
//    return bit;
//}
//
//uint8_t ds18b20_read_byte(void){
//    uint8_t data = 0;
//    for (uint8_t i = 0; i <= 7; i++)
//        data += ds18b20_read_bit() << i;
//    return data;
//}
//
//void ds18b20_write_bit(uint8_t bit){
//    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BR11); // Низкий уровень
//    delay_micro(bit ? 3 : 65);
//    SET_BIT(GPIOB->BSRR, GPIO_BSRR_BS11); // Высокий уровень
//    delay_micro(bit ? 65 : 3);
//}
//
//void ds18b20_write_byte(uint8_t dt){
//    for (uint8_t i = 0; i < 8; i++)
//    {
//        ds18b20_write_bit(dt >> i & 1);
//        delay_micro(5);
//    }
//}
//
//void ds18b20_start_measuring_cmd(){
//    ds18b20_reset();
//
//    ds18b20_write_byte(0xCC);
//
//    ds18b20_write_byte(0x44); // Команда начала измерения
//}
//
//void ds18b20_read_temp(uint8_t *data){
//    uint8_t i;
//    ds18b20_reset();
//
//    ds18b20_write_byte(0xCC);
//
//    ds18b20_write_byte(0xBE); // Команда передачи данных
//    for(i=0;i<8;i++)
//    {
//        data[i] = ds18b20_read_byte();
//    }
//}
//
//uint8_t ds18b20_set_settings(void){
//    if (ds18b20_reset()) return 1;
//
//    ds18b20_write_byte(0xCC);
//    ds18b20_write_byte(0x4E); // Команда записи регистров
//
//    ds18b20_write_byte(0x64); // 100 градусов
//    ds18b20_write_byte(0x9E); // -30 градусов
//    ds18b20_write_byte(0x7F); // Максимальная точность (Счет происходит за 700 млсек)
//}
//
//char ds18b20_get_temp_sign(uint16_t dt){
//    if (dt&(1<<11)) return '+'; else return '-'; // Если в байте знака есть единица, то возвращаем 1 - как плюсовое значение
//}
//
//float ds18b20_сonvert(uint16_t dt){
//    float t;
//    t = (float) ((dt&0x07FF)>>4);
//    t += (float)(dt&0x000F) / 16.0f;
//    return t;
//}
//
