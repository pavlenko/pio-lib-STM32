### Master TX DMA
- configure & enable DMA channel
- set DMAEN = 1
- send START, wait SB is set & clear it
- send slave address, wait ADDR is set & clear it
- wait until DMA TC = 1, disable DMA channel, clear DMA TC
- wait until BTF = 1, send STOP, wait STOPF cleared

### Master RX DMA
- configure & enable DMA channel
- set DMAEN = 1, set LAST = 1
- send START, wait SB is set & clear it
- send slave address, wait ADDR is set & clear it
- wait until DMA TC = 1, disable DMA channel, clear DMA TC
- send STOP, wait STOPF cleared

### Slave TX DMA
- enable IRQ (for handle ADDR & AF)
- ADDR interrupt: configure DMA channel, clear ADDR
- AF interrupt: disable DMA channel

### Slave RX DMA
- enable IRQ (for handle ADDR & STOPF)
- ADDR interrupt: configure DMA channel, clear ADDR
- STOPF interrupt: disable DMA channel

-----------------------------------------------------------------

# I2C workflow TODO....

### Master send

- wait busy (BUSY flag)
- send start
  - check if success (SB flag)
- send dev address (W)
  - check if success (ADDR & ADD10 flags)
- send byte(s)
  - check if success (BTF flag)
- send stop

### Master recv

- wait busy (BUSY flag)
- send start
  - check if success (SB flag)
- send dev address (R)
  - check if success (ADDR & ADD10 flags)
- recv byte(s)
  - check if success (BTF flag)
- send stop

### Mem set

- wait busy
- enable if any
- send start
  - check if success (SB flag)
- send dev address (W)
  - check if success (ADDR & ADD10 flags)
- send reg address
  - check if success (TXE flag)
- send byte(s)
  - check if success (TXE flag)
- send stop

### Mem get

- wait busy
- enable if any
- send start
  - check if success (SB flag)
- send dev address (W)
  - check if success (ADDR & ADD10 flags)
- send reg address
  - check if success (TXE flag)
- send restart
  - check if success (SB flag)
- send dev address (R)
  - check if success (ADDR & ADD10 flags)
- recv byte(s)
  - check if success (RXNE & BTF flags)
- send stop

### Slave send

- enable ACK
- wait address (ADDR & ADD10 flags)
- send byte(s)
  - check if success (TXE flag)
- disable ACK

### Slave recv

- enable ACK
- wait address (ADDR & ADD10 flags)
- recv byte(s)
  - check if success (RXNE & BTF flags)
- wait stop
- disable ACK

### DMA TX

- success:
  - disable IRQ (?)
  - disable ACK
  - send STOP
  - disable DMA (?)
  - execute callaback
- error:
  - disable ACK
  - send STOP
  - disable DMA (?)
  - execute callaback

### DMA RX

- send size - 1
- success:
  - disable IRQ (?)
  - disable ACK
  - disable DMA (?)
  - receive last byte
  - send STOP
  - execute callback
- error:
  - disable ACK
  - send STOP
  - disable DMA (?)
  - execute callaback