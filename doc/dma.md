### Abort procedure

- Disable all DMA IRQ
- Disable DMA Channel/Stream
- Wait for the Channel/Stream to be Disabled
- Clear any pending IRQ flags for Channel/Stream

After that you can use DMA channel counter register if needed
