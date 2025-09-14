#include <stm32/dev/clock.hpp>

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
