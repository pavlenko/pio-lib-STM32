#include <stm32/clock.hpp>

/**
 * @brief Dummy main function
 */
int main(void)
{
    using namespace STM32;

    while (true)
    {
        asm volatile ("nop");
    }
    
}
