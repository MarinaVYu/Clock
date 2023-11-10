//
// Created by petry on 09.10.2023.
//

#include "USART.h"
#define SIZE_OF_BUFFER 20

static usart_callback clb = NULL;


void init_usart1() {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Подача тактирования с шины APB2 на USART1
    //TX - PA9 - output - синий
    //RX - PA10 - input - фиолетовый
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN|RCC_APB2ENR_AFIOEN;// Включавем порт C GPIO, Включение альтернативной фуекции GPIO
//    RCC->APB2RSTR |= RCC_APB2RSTR_IOPARST;
//    RCC->APB2RSTR &= ~RCC_APB2RSTR_IOPARST;//Ресетим GPIO
    //Настройка ноги PA9 TX
    GPIOA->CRH &= ~GPIO_CRH_MODE9;//Обнуление регистра
    GPIOA->CRH |= GPIO_CRH_MODE9_0; //Нога PA9 на output
    GPIOA->CRH &= ~GPIO_CRH_CNF9;//Обнуление регистра
    GPIOA->CRH |= GPIO_CRH_CNF9_1;//Alternate function output Push-pull
    //Настройка ноги PA10 RX
    GPIOA->CRH &= ~GPIO_CRH_MODE10;//Обнуление регистра
    GPIOA->CRH |= GPIO_CRH_CNF10_0;//Floating input (reset state)

    //Настройка USART
    USART1->CR1 |= USART_CR1_UE; //Включение USART
    //Word lenth = 8 bit
    // 1 Stop bit
//    USART1->CR3 |= USART_CR3_DMAT; //DMA enable transmitter
    USART1->BRR = 4U<<USART_BRR_DIV_Mantissa_Pos;
    USART1->BRR |= 5U<<USART_BRR_DIV_Fraction_Pos;//Fck=8MHz, BAUD = 115 200bod, USARTDIV = 4,3125
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE; //Transmitter enable and Receiver is enabled and begins searching for a start bit

    //The TX pin is in low state during the start bit. It is in high state during the stop bit.
    // When the transmitter is enabled and nothing is to be transmitted, the TX pin is at high level

    USART1->CR1 |= USART_CR1_TXEIE | USART_CR1_RXNEIE;//TXE interrupt enable and An USART interrupt is generated

    NVIC->IP[USART1_IRQn] = 0xE0;//Установка пониженного приотритета для прерывания от USART1
    NVIC_EnableIRQ(USART1_IRQn);
}
uint8_t buffer[SIZE_OF_BUFFER] = {0};
int i = 0;//счетчик строки для записи - хвост буфера
int j = 0;//счетчик-указатель строки для передачи - голова буфера
int write_to_buffer(uint8_t value){
    //При разных взаиморасположениях указателей выявляются разные условия переполнения буффера
    if (i<j){
        if(i - j + 1 == SIZE_OF_BUFFER){
            return 0;
        }
    }
    else if (i>j){
        if (i + SIZE_OF_BUFFER - j + 1 == SIZE_OF_BUFFER){
            return 0;
        }
    }
    if (i<SIZE_OF_BUFFER){
        buffer[i] = value;
        i++;

    }
    else {
        i=0;
        buffer[i] = value;
        i++;
    }
    USART1->CR1 |= USART_CR1_TXEIE;//Включение прерывания от USART TXE
    return 1;
}

void transmit_from_buffer_to_usart(){
    if (i==j) {
        USART1->CR1 &= ~USART_CR1_TXEIE;//Выключение прерывания от USART по TX
        return;
    }

    if (j<SIZE_OF_BUFFER) {
        USART1->DR = buffer[j];
        buffer[j] = 0;
        j++;
    } else{
        j = 0;
        USART1->DR = buffer[j];
        buffer[j] = 0;
        j++;
    }
}



void USART1_IRQHandler(){
    if (USART1->SR & USART_SR_TXE_Msk){
        transmit_from_buffer_to_usart();
    }
    if (USART1->SR & USART_SR_RXNE_Msk){
        char data_from_console = USART1->DR;
        if (clb != NULL){
            clb(data_from_console);
        }
    }
}

void usart_add_callback(usart_callback cbk){
    clb = cbk;
}