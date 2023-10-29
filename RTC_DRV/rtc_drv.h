//
// Created by petry on 31.08.2023.
//

#ifndef TEST_RTC_DRV_H
#define TEST_RTC_DRV_H

typedef void (*rtc_callback) (void); // создание специального типа переменных для подачи в функцию add_callback, который является указателем на фукцию, берущую ничего и возвращающую ничего

void rtc_init (); //настройка rtc
void add_callback (rtc_callback cbk); // callback для передачи rtc_second в clock



#endif //TEST_RTC_DRV_H
