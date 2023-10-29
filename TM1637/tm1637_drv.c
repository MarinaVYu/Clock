//
// Created by petry on 09.07.2023.
//

#include <stm32f100xb.h>
#include "tm1637_drv.h"
#include "stm32f1xx_hal.h"

typedef struct {
    uint8_t packet_data[5];
    uint8_t packet_size;
} Packet;

#define BUFFER_SIZE 100
#define NONE_PWM_FORCED_HIGH (TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_2)
#define PWM                  (TIM_CCMR1_OC1M)
#define PWM_RESET            (~TIM_CCMR1_OC1M)

#define COMPARE_ON      TIM2->DIER = TIM_DIER_CC1IE; \
                        TIM2->SR = 0;
#define OVERFLOW_ON     TIM2->DIER = TIM_DIER_UIE; \
                        TIM2->SR = 0;

Packet send_data_buff[BUFFER_SIZE] = {0};

uint8_t real_size_of_buff = 0;

static void tm1637_display_char(uint8_t num, uint8_t pos, uint8_t points_on);
static void remove_first_from_buf(Packet* buf_p, uint8_t* buf_size);
static void put_to_buf (Packet* buf, uint8_t* size, Packet* data);

void tm1637_init() {

    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_AFIOEN;

    RCC->APB2RSTR |= 1U << 4U;
    RCC->APB2RSTR &= ~(1U << 4U);

    GPIOC->CRH = GPIO_CRH_MODE8_0; // настраиваем на выход ногу С8
    GPIOC->ODR = GPIO_ODR_ODR8; // настраиваем DIO на 1
    GPIOA->CRL = GPIO_CRL_MODE0_0 | GPIO_CRL_CNF0_1; // настраиваем на ноге А0 альтернативную функцию и настраиваем ее на выход

    uint32_t freq = HAL_RCC_GetSysClockFreq();
    uint16_t period = freq / 8000U;

    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // включение тактирования с шины apb1 на таймер 2
    TIM2->ARR = period;
    TIM2->CCR1 = period / 2; // настройка прерывания на половину периода таймера
    TIM2->DIER = TIM_DIER_CC1IE; // подключаем прерывание по сравнению на канал С1
    HAL_NVIC_EnableIRQ(TIM2_IRQn); // разрешили прерывание от таймера 2

    TIM2->CCMR1 = NONE_PWM_FORCED_HIGH; // устанавливаем единицу на выходном канале таймера
    TIM2->CCER = TIM_CCER_CC1E;// включили канал, сединяющий таймер и мультиплексор

    TIM2->CR1 |= TIM_CR1_CEN; // включение таймера

}

void tm1637_clear_all() {
    Packet set_disp_packet = {
            .packet_data = {0x40,}, //Настройка дисплея на выведение
            .packet_size = 1
    };
    put_to_buf(send_data_buff, &real_size_of_buff, &set_disp_packet);

    Packet clear_packet = {
            .packet_data = {
                    0xC0U, // Настройка номера индикатора, используя заданную позицию
                    0,0,0,0 // Передаём нули, чтобы погасить все сегменты индикатора
                    },
            .packet_size = 5
    };
    put_to_buf(send_data_buff, &real_size_of_buff, &clear_packet);

}
void tm1637_display(uint16_t num, uint8_t points_on) {

    uint8_t third = num / 1000; // Выделяем тысячи из полученного числа
    uint16_t ost = num % 1000; // Записываем остаток в отдельную переменную
    uint8_t second = ost / 100; // Выделяем сотни из остатка
    ost = ost % 100;// Записываем остаток в отдельную переменную
    uint8_t first = ost / 10; // Выделяем десятки из остатка
    uint8_t zero = ost % 10;// Записываем остаток как единицы
    tm1637_display_char(third, 0,0);
    tm1637_display_char(second, 1,points_on);
    tm1637_display_char(first,2, 0);
    tm1637_display_char(zero,3,0);
}

uint8_t tm1637_mass[26] = {
        [0] = 0x3F,
        [1] = 0x06,
        [2] = 0x5B,
        [3] = 0x4F,
        [4] = 0x66,
        [5] = 0x6D,
        [6] = 0x7D,
        [7] = 0x07,
        [8] = 0x7F,
        [9] = 0x6F,
        0x77,//A
        0x7D,//Б
        0x7F,//В
        0x31,//Г
        0x5F,//Д
        0x79,//E
        0x4F,//З
        0x76,//H
        0x3F,//O
        0x37,//П
        0x73,//P
        0x39,//C
        0x6E,//У
        0x66,//Ч
        0x7C,//Ь
        0x00//
};

void tm1637_display_on() {
    Packet on_disp_packet = {
            .packet_data = {0x8F,},
            .packet_size = 1
    };
    put_to_buf(send_data_buff, &real_size_of_buff, &on_disp_packet);
}

static void tm1637_display_char(uint8_t num, uint8_t pos, uint8_t points_on) {
    Packet set_disp_packet = {
            .packet_data = {0x44U,},
            .packet_size = 1
    };
    put_to_buf(send_data_buff, &real_size_of_buff, &set_disp_packet);
    Packet set_pos_packet = {
            .packet_data = {0xC0U | pos, tm1637_mass[num] | (points_on << 7U) },
            .packet_size = 2
    };
    put_to_buf(send_data_buff, &real_size_of_buff, &set_pos_packet);
}

static void put_to_buf (Packet* buf, uint8_t* size, Packet* data) {
    buf[*size] = *data;
    *size = *size + 1;
}

typedef enum States_enum {
    IDLE,
    START,
    OUTPUT,
    INPUT,
    INPUT_READ,
    STOP_REOUTPUT_1,
    STOP_FORCED_CLK_2,
    STOP_DIO_HIGH_3,
} States;

States state = IDLE;

uint8_t curr_packet_index = 0;

uint8_t num_of_bit = 1;

uint8_t read_dio = 0;

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_CC1IF_Msk) {
        switch (state) {
            case IDLE:
                if (real_size_of_buff != 0) {
                    state = START;
                }
                break;
            case START:
                GPIOC->ODR &= ~GPIO_ODR_ODR8; // DIO -> 0
                TIM2->CCMR1 |= PWM; // включаем PWM
                OVERFLOW_ON // включаем прерывание по переполнению
                state = OUTPUT;
                break;
            case INPUT_READ:
                read_dio = GPIOC->IDR & GPIO_IDR_IDR8_Msk;
                curr_packet_index++;
                OVERFLOW_ON // включаем прерывание по переполнению
                if(curr_packet_index == send_data_buff[0].packet_size) {
                    curr_packet_index = 0;

                    state = STOP_REOUTPUT_1;
                }
                else {
                    state = OUTPUT;
                }

                break;

            case STOP_FORCED_CLK_2:
                TIM2->CCMR1 &= PWM_RESET; // Зануляем регистр перед наложением маски
                TIM2->CCMR1 |= NONE_PWM_FORCED_HIGH; // устанавливаем единицу на выходном канале таймера
                OVERFLOW_ON // включаем прерывание по переполнению
                state = STOP_DIO_HIGH_3;
                break;
        }
    }
    if (TIM2->SR & TIM_SR_UIF_Msk){
        switch (state) {
            case OUTPUT:
                GPIOC->CRH |=  GPIO_CRH_MODE8_0; // DIO -> Output
                uint8_t bit = (send_data_buff[0].packet_data[curr_packet_index] & (1U << (num_of_bit - 1U))) >> (num_of_bit - 1); // с помощью маски выделяем i-1й бит из data и ставим его в нулевую позицию
                if (bit == 1) {
                    GPIOC->ODR |= GPIO_ODR_ODR8_Msk; // DIO -> 1
                }
                else if (bit == 0) {
                    GPIOC->ODR &= ~GPIO_ODR_ODR8_Msk; // DIO -> 0
                }
                num_of_bit++;
                if (num_of_bit >= 9) {
                    num_of_bit = 1;
                    state = INPUT;
                }
                break;

            case INPUT:
                GPIOC->ODR &= ~GPIO_ODR_ODR8_Msk; // в регистр output выставляем 0
                GPIOC->CRH &= ~GPIO_CRH_MODE8_0;// линия данных в положение input
                COMPARE_ON // подключаем прерывание по сравнению на канал С1
                state = INPUT_READ;

                break;

            case STOP_REOUTPUT_1:
                GPIOC->CRH |=  GPIO_CRH_MODE8_0; // настраиваем линию данных на output
                COMPARE_ON; // подключаем прерывание по сравнению на канал С1
                state = STOP_FORCED_CLK_2;

                break;
            case STOP_DIO_HIGH_3:
                GPIOC->ODR |= GPIO_ODR_ODR8_Msk; // DIO -> 1
                remove_first_from_buf(send_data_buff, &real_size_of_buff);
                COMPARE_ON; // подключаем прерывание по сравнению на канал С1
                state = IDLE;
                break;
        }
    }


    TIM2->SR = 0;
}

static void remove_first_from_buf(Packet* buf_p, uint8_t* buf_size) {
    for(uint8_t i = 0; i < (*buf_size - 1); i++)
        buf_p[i] = buf_p[i+1];
    *buf_size -= 1;
}