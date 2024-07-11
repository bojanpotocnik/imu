#include "uart.h"


// Define static variables outside the class definition
constexpr uint8_t UART::instances[];
uint8_t UART::total_instances = 0;
