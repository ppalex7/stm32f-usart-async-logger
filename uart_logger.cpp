#include <cstdio>

#include "stm32f030x6.h"

#include <uart_logger.hpp>

#define BUFFER_MESSAGES_COUNT 16u
#define USART_DR (USART1_BASE + 0x01)

static char g_uart_tx_buffer[128];
static volatile uint16_t length = 0;
// 2-byte index produces less code for array addressing;
static volatile uint16_t idx = 0;
static volatile uint16_t drops = 0;
static const char *msg_format[BUFFER_MESSAGES_COUNT] = { 0 };
static volatile short msg_arg[BUFFER_MESSAGES_COUNT];

inline static void _initialize_uart(const uint32_t usart_brr);
inline static void _initialize_dma();

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
    DMA1_Channel2->CPAR = (uint32_t) (&(USART1->TDR));

    // Configure the memory address
    DMA1_Channel2->CMAR = reinterpret_cast<uint32_t>(g_uart_tx_buffer);

    // priority level: medium
    // memory increment mode
    // data transfer direction: read from memory
    // transfer complete interrupt enable
    DMA1_Channel2->CCR = DMA_CCR_PL_0 | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
}

void uart_log(const char *const fmt, uint16_t arg) {
    bool restore_interrupts = 0;
    if (*fmt == '\0') {
        // reject empty string
        return;
    }
    if (idx > BUFFER_MESSAGES_COUNT - 1) {
        // buffer is full, drop message
        drops++;
        return;
    }

    if (__get_PRIMASK() == 0) {
        restore_interrupts = 1;
        __disable_irq();
    }

    // begin critical section
    msg_format[idx] = fmt;
    msg_arg[idx] = arg;
    idx++;
    // end critical section

    if (restore_interrupts) {
        __enable_irq();
    }
}

void on_dma_log_transfer_complete() {
    uint16_t i;
    uint8_t max_idx;
    bool restore_interrupts = 0;

    if (DMA1->ISR & DMA_ISR_TCIF2) {
        if (__get_PRIMASK() == 0) {
            restore_interrupts = 1;
            __disable_irq();
        }

        // begin critical section
        max_idx = (uint8_t) ((idx > BUFFER_MESSAGES_COUNT - 1) ? BUFFER_MESSAGES_COUNT - 1 : idx);
        if (idx == 1) {
            msg_format[0] = 0;
            msg_arg[0] = 0;
        } else {
            for (i = 0; i < max_idx; i++) {
                msg_format[i] = msg_format[i + 1];
                msg_arg[i] = msg_arg[i + 1];
            }
        }
        idx--;
        if (drops && idx < (BUFFER_MESSAGES_COUNT - 1) / 2) {
            uart_log("...truncated %d messages\n", drops);
            drops = 0;
        }
        length = 0;
        // end critical section

        if (restore_interrupts) {
            __enable_irq();
        }

        DMA1->IFCR = DMA_IFCR_CTCIF2;
    }
}

void process_buffered_logs() {
    // return if busy or has no messages
    if (length || !msg_format[0]) {
        return;
    }

    length = (unsigned char) std::snprintf(g_uart_tx_buffer, sizeof(g_uart_tx_buffer), msg_format[0], msg_arg[0]);

    // disable channel
    DMA1_Channel2->CCR &= (~DMA_CCR_EN_Msk);

    DMA1_Channel2->CNDTR = length;

    // trigger UART
    USART1->ICR |= USART_ICR_TCCF;

    // enable channel
    DMA1_Channel2->CCR |= DMA_CCR_EN;
}

