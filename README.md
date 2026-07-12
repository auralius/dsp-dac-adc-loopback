![](./loopback.png)

# Hardware Setup: STM32 DAC–ADC Loopback Filter Test

## Board

The experiment uses an **STM32U585CIU6 mini core board** programmed with the Arduino framework through PlatformIO.

The board generates test sine waves using its DAC pins, reads them back using ADC pins, runs the filter in firmware, and sends the measured input/output samples to the PC over USB serial.

## Loopback Wiring

The current loopback wiring is:

```text
PA4 / A4  -> PA0 / A0
PA5 / A5  -> PA1 / A1
GND       -> GND
```

PA4 and PA5 are used as two DAC outputs.  
A0 and A1 are used as ADC inputs.
