#ifndef __UART_LOGGER_H
#define __UART_LOGGER_H

/* Maximum formatted message length limited to 127 symbols:
 * printf behaviour is strange - it repeatedly outputs first character.
 * Please do not direct use prinf/putchar methods.
 */
#define log(a) logf(a, 0)
void logf(@near char* fmt, unsigned short arg);
void configure_logger_peripheral(unsigned short clock_divider);
void on_dma_log_transfer_complete(void);
void process_buffered_logs(void);

#endif