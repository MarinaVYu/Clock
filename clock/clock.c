//
// Created by petry on 28.10.2023.
//

#include "clock.h"
#include "rtc_drv.h"
#include "tm1637_drv.h"
#include <stm32f100xb.h>
#include "stm32f1xx_hal.h"

uint16_t time = 0;
static void clock_tick();

void clock_init(void){
    rtc_init();
    tm1637_init();
    tm1637_display_on();
    tm1637_clear_all();
    tm1637_display(0,1);
    add_callback(clock_tick);

}

static void clock_tick(){
    time++;
    uint8_t tim_min = time/ 60;
    uint8_t tim_sec = time % 60;

    if ((time % 2) == 0U){

        tm1637_display(tim_min*100 + tim_sec,1);

    } else{

        tm1637_display(tim_min*100 + tim_sec,0);
    }

}