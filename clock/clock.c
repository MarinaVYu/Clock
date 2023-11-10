//
// Created by petry on 28.10.2023.
//

#include "clock.h"
#include "rtc_drv.h"
#include "tm1637_drv.h"
#include <stm32f100xb.h>
#include "stm32f1xx_hal.h"
#include "USART.h"

uint16_t time = 0;
uint16_t buffer_for_data_from_usart = 0;
static uint8_t i = 0;

static void clock_tick();
static void receive(unsigned char data_from_usart);



void clock_init(void){
    rtc_init();
    tm1637_init();
    tm1637_display_on();
    tm1637_clear_all();
    tm1637_display(0,1);
    add_callback(clock_tick);
    usart_add_callback(receive);

}

static void clock_tick(){
    time--;
    uint8_t tim_min = time/ 60;
    uint8_t tim_sec = time % 60;

    if ((time % 2) == 0U){

        tm1637_display(tim_min*100 + tim_sec,1);

    } else{

        tm1637_display(tim_min*100 + tim_sec,0);
    }

}

static void receive(unsigned char data_from_usart){
    if (i == 0){
        buffer_for_data_from_usart = data_from_usart * 60;
        i++;
    }else{
        buffer_for_data_from_usart = buffer_for_data_from_usart + data_from_usart ;
        i = 0;
        time = buffer_for_data_from_usart + 1;
        clock_tick();
    }

}
