#include "stm32f030x6.h"

#include <uart_logger.hpp>
#include <CircularMessageBuffer.hpp>

#ifndef BUFFER_MESSAGES_COUNT
#define BUFFER_MESSAGES_COUNT 16u
#endif

#ifndef LOG_MESSAGE_MAX_LEN
#define LOG_MESSAGE_MAX_LEN 128
#endif

inline static void _initialize_uart(const uint32_t usart_brr);
inline static void _initialize_dma();

static CircularMessageBuffer<uint16_t, BUFFER_MESSAGES_COUNT, LOG_MESSAGE_MAX_LEN> g_buffer;

void configure_logger_peripheral(const uint32_t usart_brr) {
    _initialize_uart(usart_brr);
    _initialize_dma();
    NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

inline static void _initialize_uart(const uint32_t usart_brr) {
    // Feed clock to GPIOA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Configure PA2 to alternate function
    GPIOA->MODER |= GPIO_MODER_MODER2_1;
    // Select AF1 (USART1_TX) for PA2
    GPIOA->AFR[0] |= (0b0001 << GPIO_AFRL_AFSEL2_Pos);

    // Feed clock to USART1
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Set USART baud rate
    USART1->BRR = usart_brr;

    // Enable transmitter
    // Transmitter enable
    // USART enable
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    // Enable DMA mode for transmission
    USART1->CR3 = USART_CR3_DMAT;

    // Polling idle frame transmission
    while ((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
        ;
}

inline static void _initialize_dma() {
    // Feed clock to DMA1
    RCC->AHBENR |= RCC_AHBENR_DMAEN;

    // set destination peripheral address
    DMA1_Channel2->CPAR = reinterpret_cast<uint32_t>(&(USART1->TDR));

    // Configure the memory address
    DMA1_Channel2->CMAR = g_buffer.getBufferAddress();

    // priority level: medium
    // memory increment mode
    // data transfer direction: read from memory
    // transfer complete interrupt enable
    DMA1_Channel2->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
}

void uart_log(const char *const fmt, uint16_t arg) {
    g_buffer.enqueue(fmt,  arg);
}

void on_dma_log_transfer_complete() {
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        g_buffer.acknowledge();

        DMA1->IFCR = DMA_IFCR_CTCIF2;
    }
}

void process_buffered_logs() {
    int new_length = g_buffer.dispatch();
    if (new_length <= 0) {
        return;
    }

    // disable channel
    DMA1_Channel2->CCR &= (~DMA_CCR_EN_Msk);

    DMA1_Channel2->CNDTR = static_cast<uint32_t>(new_length);

    // trigger UART
    USART1->ICR |= USART_ICR_TCCF;

    // enable channel
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

