# Appareo

This repository contains firmware for an STM32 microcontroller demo using the HTS1510 pressure and temperature sensor. Two PWM outputs drive 4-20 mA loops for pressure (0–175 psia) and temperature (-60 °C to 50 °C).

## Building

Open the project in STM32CubeIDE or run `make` if a Makefile is available. The firmware depends on the STM32 HAL libraries.

## Hardware

- HTS1510 sensor on I2C1 (PB6/PB7)
- PWM output TIM1_CH1 on PA5 for temperature
- PWM output TIM3_CH1 on PA6 for pressure
- UART1 at 115200 baud for optional logging
- Independent watchdog with ~4 s timeout

## Logging

Define `ENABLE_LOGGING=1` at compile time to emit diagnostic UART messages. Undefine it for production builds to save bandwidth.
