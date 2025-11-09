### IRQ handler

- Check if TE
  - Disable TE IRQ
  - Clear flag TE
  - Error |= TE
- Check if FE
  - Clear flag FE
  - Error |= FE
- Check if DME
  - Clear flag DME
  - Error |= DME
- Check if HT
  - Clear flag HT
  - If not circular - Disable HT IRQ
  - Execute HT callback
- Check if TC
  - Clear flag TC
  - If not circular - Disable TC IRQ
  - Execute TC CALLBACK
- If error
  - If has TE error
    - Disable DMA Channel/Stream
    - Wait for the Channel/Stream to be Disabled
  - Execute error callback

### Abort procedure

- Disable all DMA IRQ
- Disable DMA Channel/Stream
- Wait for the Channel/Stream to be Disabled
- Clear any pending IRQ flags for Channel/Stream
