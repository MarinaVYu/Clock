//
// Created by petry on 09.10.2023.
//

#ifndef TEST_USART_H
#define TEST_USART_H
#include "stm32f1xx_hal.h"

typedef void (*usart_callback) (unsigned char); // создание специального типа переменных для подачи в функцию add_callback, который является указателем на фукцию, берущую ничего и возвращающую ничего


void init_usart1();
int write_to_buffer(uint8_t value);
void usart_add_callback(usart_callback cbk);

#endif //TEST_USART_H
