#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED

#include <HardwareSerial.h>
#include <driver/uart.h> //< UART_NUM_* constants


class UART : public HardwareSerial
{
public:
    const uint8_t instance = total_instances;
    const int8_t pin_rx;
    const int8_t pin_tx;
    const uint32_t baud;
    const uint32_t rx_timeout;

    /**
     * @brief Construct a new UART object
     *
     * The instance number is automatically assigned based on the number of instances created.
     *
     * @param rx_pin      RX pin number or -1 if not used.
     * @param tx_pin      TX pin number or -1 if not used.
     * @param baud        Baud rate for the UART peripheral.
     * @param rx_timeout  Timeout in milliseconds for read operations.
     *
     * @see HardwareSerial::HardwareSerial
     * @see HardwareSerial::begin
     */
    UART(int8_t rx_pin, int8_t tx_pin, uint32_t baud, uint32_t rx_timeout)
        : HardwareSerial(instances[total_instances]),
          pin_rx(rx_pin),
          pin_tx(tx_pin),
          baud(baud),
          rx_timeout(rx_timeout)
    {
        // Increment instance count after initializing `HardwareSerial` and `instance`
        total_instances++;
    }

    UART(int8_t rx_pin, int8_t tx_pin, uint32_t baud, uint32_t rx_timeout, size_t rx_buffer_size,
         size_t tx_buffer_size = 128)
        : UART(rx_pin, tx_pin, baud, rx_timeout)
    {
        setRxBufferSize(rx_buffer_size);
        setTxBufferSize(tx_buffer_size);
    }

    void init()
    {
        HardwareSerial::begin(baud, SERIAL_8N1, pin_rx, pin_tx, false, 1000);
        setTimeout(rx_timeout);
    }

private:
    static constexpr uint8_t instances[] = {UART_NUM_0, UART_NUM_1, UART_NUM_2};
    static uint8_t total_instances;
};

#endif // UART_H_INCLUDED
