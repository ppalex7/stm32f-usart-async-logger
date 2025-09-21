#ifndef __UART_LOGGER_HPP
#define __UART_LOGGER_HPP

void uart_log(const char *const fmt, uint16_t arg = 0);
void configure_logger_peripheral(const uint32_t usart_brr);
void on_dma_log_transfer_complete();
void process_buffered_logs();

#endif
