//
// Created by petry on 09.07.2023.
//

#include <stdint-gcc.h>

#ifndef TEST_TM1637_DRV_H
#define TEST_TM1637_DRV_H

void tm1637_display_on();
void tm1637_display(uint16_t num, uint8_t points_on);
void tm1637_clear_all();
void tm1637_init();

#endif //TEST_TM1637_DRV_H