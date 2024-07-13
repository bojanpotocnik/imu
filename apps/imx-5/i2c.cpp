#include "i2c.h"


// Define static variables outside the class definition
constexpr uint8_t I2C::instances[];
uint8_t I2C::total_instances = 0;
