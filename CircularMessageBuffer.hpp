#ifndef CIRCULARMESSAGEBUFFER_HPP_
#define CIRCULARMESSAGEBUFFER_HPP_

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <cmsis_gcc.h>

template<typename T, std::size_t MESSAGES_COUNT, std::size_t MAX_LENGTH>
class CircularMessageBuffer {
private:

    const char *msg_format[MESSAGES_COUNT] = { 0 };
    T msg_arg[MESSAGES_COUNT];

    volatile std::size_t idx_write = 0;
    std::size_t idx_read = 0;

    volatile char tx_buffer[MAX_LENGTH];

    uint16_t drops = 0;
    std::size_t size = 0;

    static std::size_t next_index(const std::size_t current_index) {
        if (current_index == (MESSAGES_COUNT - 1)) {
            return 0;
        }
        return current_index + 1;
    }

public:

    uint32_t getBufferAddress() const {
        return reinterpret_cast<uint32_t>(tx_buffer);
    }

    void enqueue(const char *const fmt, T arg) {
        if (fmt[0] == '\0') {
            // reject empty string
            return;
        }

        if (msg_format[idx_write][0] != '\0') {
            // buffer is full, drop message
            drops++;
            return;
        }

        bool restore_interrupts = __get_PRIMASK() == 0;
        __disable_irq();

        std::size_t idx = idx_write;
        msg_format[idx] = fmt;
        msg_arg[idx] = arg;
        idx_write = next_index(idx);
        size++;

        if (restore_interrupts) {
            __enable_irq();
        }
    }

    int dispatch() {
        // return if busy or has no messages
        if (tx_buffer[0] != 0 || msg_format[idx_read][0] == '\0') {
            return 0;
        }

        int length = std::snprintf(const_cast<char*>(tx_buffer), sizeof(tx_buffer), msg_format[idx_read],
                msg_arg[idx_read]);

        bool restore_interrupts = __get_PRIMASK() == 0;
        __disable_irq();

        if (length <= 0) {
            // in case of error occurred
            acknowledge();
        }

        msg_format[idx_read] = "";
        idx_read = next_index(idx_read);
        size--;

        if (drops && size < (MESSAGES_COUNT - 1) / 2) {
            enqueue("...truncated %d messages\n", drops);
            drops = 0;
        }

        if (restore_interrupts) {
            __enable_irq();
        }

        return length;
    }

    void acknowledge() {
        tx_buffer[0] = '\0';
    }
};

#endif /* CIRCULARMESSAGEBUFFER_HPP_ */
