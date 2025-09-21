# STM32 C++ header based HAL library

### Modules

- [x] Clock
- [x] Flash
- [ ] DMA
- [x] EXTI
- [x] GPIO
- [ ] I2C
- [ ] SPI
- [ ] UART

### Clock configuration

Oscilators

- Disable oscilator if needed
- Configure source, dividers, multipliers
- Enable oscilator

CPU & bus clocks

- Increase flash latency if needed
- Configure AHBClock divider
- Configure SysClock source
- Decrease flash latency if needed
- Configure APB1Clock divider
- Configure APB2Clock divider
- Update SystemCoreClock value