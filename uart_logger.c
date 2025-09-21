#include "uart_logger.h"
#include "stm8l15x.h"
#include <stdio.h>
#include <processor.h>

#define BUFFER_MESSAGES_COUNT 16u
#define USART_DR (USART1_BASE + 0x01)

@near unsigned char g_uart_tx_buffer[128];
static volatile uint16_t length = 0;
// 2-byte index produces less code for array addressing;
static volatile uint16_t idx = 0;
static volatile uint16_t drops = 0;
static volatile char *msg_format[BUFFER_MESSAGES_COUNT] = {0};
static volatile short msg_arg[BUFFER_MESSAGES_COUNT];

void configure_logger_peripheral(unsigned short clock_divider)
{
    // Feed clock to USART1
    CLK->PCKENR1 |= CLK_PCKENR1_USART1;

    USART1->BRR2 = (uint8_t)(clock_divider & 0x0F);
    USART1->BRR1 = (uint8_t)(clock_divider >> 4);
    // Enable transmitter
    USART1->CR2 |= USART_CR2_TEN;
    // Enable DMA mode for transmission
    USART1->CR5 |= USART_CR5_DMAT;

    // Feed clock to DMA1
    CLK->PCKENR2 |= CLK_PCKENR2_DMA1;
    // set destination peripheral address
    DMA1_Channel1->CPARH = (uint8_t)(USART_DR >> 8);
    DMA1_Channel1->CPARL = (uint8_t)(USART_DR);
    // set channe priority to high
    DMA1_Channel1->CSPR |= (uint8_t)(0b10 << 4);
    // set memory increment mode and data transfer direction from memory to peripheral
    DMA1_Channel1->CCR |= (DMA_CCR_IDM | DMA_CCR_DTD | DMA_CCR_TCIE);
    // DMA global enable
    DMA1->GCSR |= DMA_GCSR_GE;
}

void logf(@near char *fmt, uint16_t arg)
{
    bool restore_interrupts = 0;
    if (*fmt == '\0')
    {
        // reject empty string
        return;
    }
    if (idx > BUFFER_MESSAGES_COUNT - 1)
    {
        // buffer is full, drop message
        drops++;
        return;
    }

    if (!imask())
    {
        restore_interrupts = 1;
        disableInterrupts();
    }

    // begin critical section
    msg_format[idx] = fmt;
    msg_arg[idx] = arg;
    idx++;
    // end critical section

    if (restore_interrupts)
    {
        enableInterrupts();
    }
}

void on_dma_log_transfer_complete(void)
{
    uint16_t i;
    uint8_t max_idx;
    bool restore_interrupts = 0;

    if (DMA1_Channel1->CSPR & DMA_CSPR_TCIF)
    {
        if (!imask())
        {
            restore_interrupts = 1;
            disableInterrupts();
        }

        // begin critical section
        max_idx = (uint8_t)((idx > BUFFER_MESSAGES_COUNT - 1)
                                ? BUFFER_MESSAGES_COUNT - 1
                                : idx);
        if (idx == 1)
        {
            msg_format[0] = 0;
            msg_arg[0] = 0;
        }
        else
        {
            for (i = 0; i < max_idx; i++)
            {
                msg_format[i] = msg_format[i + 1];
                msg_arg[i] = msg_arg[i + 1];
            }
        }
        idx--;
        if (drops && idx < (BUFFER_MESSAGES_COUNT - 1) / 2)
        {
            logf("...truncated %d messages\n", drops);
            drops = 0;
        }
        length = 0;
        // end critical section

        if (restore_interrupts) {
            enableInterrupts();
        }

        DMA1_Channel1->CSPR &= (uint8_t)~DMA_CSPR_TCIF;
    }
}

void process_buffered_logs(void)
{
    // return if busy or has no messages
    if (length || !msg_format[0])
    {
        return;
    }

    printf(msg_format[0], msg_arg[0]);

    // disable channel
    DMA1_Channel1->CCR &= (uint8_t)~DMA_CCR_CE;

    // set source address
    DMA1_Channel1->CM0ARH = (uint8_t)(((uint16_t)g_uart_tx_buffer) >> (uint8_t)8);
    DMA1_Channel1->CM0ARL = (uint8_t)(((uint16_t)g_uart_tx_buffer));

    DMA1_Channel1->CNBTR = (uint8_t) length;

    // trigger UART
    USART1->SR &= (uint8_t)(~USART_SR_TC);
    // enable channel
    DMA1_Channel1->CCR |= DMA_CCR_CE;
}

char putchar(char c) {
   if (length >= sizeof(g_uart_tx_buffer) ) {
        return 0;
    }

    g_uart_tx_buffer[length] = c;
    length++;

    return c;
}

