/** serial_com.h 
 * @file serial_com.h
 * @brief Serial communication interface header for STM32 RTOS-based DirectMRAC project.
 *
 * Developed for educational and research purposes.
 *
 * @author Dr. Bharat Verma
 * @note Assistant Professor, The LNMIIT, Jaipur, India
 * @note ORCID: https://orcid.org/0000-0001-7600-7872
 * @note GitHub: https://github.com/vkbharatv
 **/



#ifndef SERIAL_COM_H
#define SERIAL_COM_H

#include "stm32f7xx_hal_uart.h"
#include<string.h>
typedef struct 
{
    UART_HandleTypeDef *huart;
    uint8_t buffer[100];
} serial_com;

void serial_init(serial_com *com, UART_HandleTypeDef *huart)
{
    com->huart = huart;
    memset(com->buffer, 0, sizeof(com->buffer));
}
void serial_print(serial_com *com, char *message) {
    if ((com == NULL) || (com->huart == NULL) || (message == NULL)) {
        return;
    }
    HAL_UART_Transmit(com->huart, (uint8_t *)message, strlen(message),
                      HAL_MAX_DELAY);

}
void serial_read(serial_com *com, char *message, uint16_t size)
{
    HAL_UART_Receive(com->huart, (uint8_t *)message, size, 1);
}

#endif /* SERIAL_COM_H */