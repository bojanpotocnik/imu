#include "uart.h"


// Define static variables outside the class definition
constexpr uint8_t UART::instances[];
// Skip UART0, which also prints debug messages
uint8_t UART::total_instances = 1;
