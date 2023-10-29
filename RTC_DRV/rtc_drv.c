//
// Created by petry on 31.08.2023.
//

#include <stm32f100xb.h>
#include "rtc_drv.h"
#include "stm32f1xx_hal.h"

static rtc_callback clb = NULL;


void rtc_init() {

    RCC->APB1ENR |= RCC_APB1ENR_BKPEN; //Включили APB1 интерфейс
    RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Включаем модуль PWR
    PWR->CR = PWR_CR_DBP; // Разрешение записи в бит RCC_BDCR_RTCSEL, отвечающий за подключение часового кварца к RTC_CLK
    RCC->BDCR |= RCC_BDCR_RTCSEL_LSE; // Подключение часового кварца к RTC_CLK
    RCC->BDCR |= RCC_BDCR_LSEON; // Подача питания на часовой кварц
    while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0U ); // Ожидание стабилизации сигнала от часового кварца, как только стабилизируется, осуществится выход из цикла
    RCC->BDCR |= RCC_BDCR_RTCEN; // Включаем RTC
    RTC->CRL |= RTC_CRL_CNF; // Разрешаем запись делителя (RTC_PRL)
    RTC->PRLL = 0x7FFFU; // Устанавливаем делитель как 32 768
    RTC->CRH |= RTC_CRH_SECIE; // Включаем прерывание от RTC_Second
    RTC->CRL &= ~RTC_CRL_CNF; // Выключаем запись делителя, начинаем загрузку
    HAL_NVIC_EnableIRQ(RTC_IRQn); // разрешили прерывание от RTC


}
void RTC_IRQHandler(void){
    if ((RTC->CRL & RTC_CRL_SECF_Msk) != 0U){
        RTC->CRL &= ~RTC_CRL_SECF_Msk;
        if (clb != NULL) {
            clb();
        }

    }
}

void add_callback(rtc_callback cbk){
    clb = cbk;
}
