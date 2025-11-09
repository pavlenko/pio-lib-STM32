### Slave DMA Event

- Disable IRQ
- Clear DMA event callback
- Disable DMA
- If state == SLAVE_TX - Execute TX_DATA callback
- If state == SLAVE_RX - Execute RX_DATA callback

### Slave IRQ Event

- Read SR2 before SR1 for prevent clear ADDR
- Check ADDR
  - Execute ADDR callback (SR2.TRA == 1 ? state = SLAVE_TX : state = SLAVE_RX)
  - Clear ADDR flag (MUST BE AFTER EXECUTE CALLBACK)
- check STOPF (Master aborts transfer)
  - Clear STOPF flag
  - Disable IRQ
  - Disable ACK
  - Disable DMA, execute DMA::abort()
  - If state == LISTEN - Stop listening:
    - Execute callback(?)
  - If state == SLAVE_RX - Stop RX:
    - Execute RX_DATA callback

### Slave DMA Abort

- Wait for STOP to be reset
- Clear DMA event callback
- Disable ACK
- Clear DMA abort callback
- Disable I2C
- If state == ABORT - Execute abort callback
- Else - Execute error callback

### Slave DMA Error

- Clear DMA event callback
- If DMA.FE - skip below
- Disable ACK
- Execute error callback

### Slave IRQ Error

- Check BERR
  - Clear BERR flag
  - Error |= BERR
- Check AF (Master aborts transfer)
  - Clear AF flag
  - If state == LISTEN - Stop listening:
    - Disable IRQ
    - Disable ACK
    - Execute callback(?)
  - If state == SLAVE_TX - Stop TX:
    - Disable IRQ
    - Disable ACK
    - Execute TX_DATA callback
    - !!! DMA disabled in error handler !!!
  - Else error |= AF
- Check OVR
  - Clear OVR flag
  - Error |= OVR
- If error
  - Disable IRQ
  - Disable DMA, execute DMA::abort()
  - Execute error callback
