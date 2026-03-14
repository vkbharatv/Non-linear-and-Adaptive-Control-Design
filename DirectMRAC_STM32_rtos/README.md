# Direct MRAC on STM32F7 (FreeRTOS)

This project implements a Direct Model Reference Adaptive Control (MRAC) speed controller on an STM32F767 using FreeRTOS. The controller drives a DC motor with PWM, reads speed via a quadrature encoder, and streams telemetry over UART.

## Highlights

- Direct MRAC with online adaptation of `Kx_hat` and `Kr_hat`
- 1 kHz control loop (`dt = 0.001 s`)
- Reference model as a first-order filter (`tau = 0.5 s`)
- Encoder speed measurement with wrap-around handling
- UART telemetry for plotting and logging

## Hardware Assumptions

- MCU: STM32F767 (from project name and device headers)
- Encoder input on `TIM1` (quadrature encoder mode)
- PWM output on `TIM2` channel 1
- H-bridge direction pins on `GPIOB` pins `7`, `10`, `11`, `14`
- UART3 used for telemetry (`huart3`)

If your board wiring differs, update the GPIO assignments and timer/UART instances in `Core/Src/main.c`.

## RTOS Tasks

- `measureTask`:
  - Reads `TIM1` encoder count
  - Computes RPM from `pulses_per_rev = 4800`
  - Applies a low-pass filter (`tau = 0.1 s`)
- `Control`:
  - Updates MRAC using current RPM
  - Toggles the setpoint between `60` and `100` RPM every `2 s`
  - Drives the motor using PWM and direction pins
- `publish`:
  - Sends CSV-formatted telemetry over UART every 100 ms

## Telemetry Format

UART prints one line per update:

```
time_s,xm,x,e,u_percent,Kx_hat,Kr_hat
```

Where:
- `xm` is the reference model output
- `x` is measured RPM
- `e = x - xm`
- `u_percent` is PWM duty in percent

## Build Options

### STM32CubeIDE

Open `DirectMRACSTM.ioc` and build as a CubeIDE project.

### CMake (arm-none-eabi-gcc)

```bash
cmake --preset Debug
cmake --build --preset Debug
```

The toolchain file is `cmake/gcc-arm-none-eabi.cmake`, and build output is placed under `build/Debug`.

## Key Files

- `Core/Src/main.c`: Tasks, peripherals, and control loop wiring
- `Core/Inc/MRAC.h`: MRAC update law and reference model
- `Core/Src/controller.c`: PID, Filters and helpers
- `Core/Inc/serial_com.h`: UART telemetry helpers

## Tuning Notes

- Adaptation gain: `gamma = 0.5`
- Initial gains: `Kx_hat = -10`, `Kr_hat = 10`
- Output saturation: `umin = -9999`, `umax = 9999`

Adjust these in `Core/Src/main.c` if you change the motor, supply voltage, or load.

## Exercise

- Implement a logging mechanism to store telemetry data on an SD card for offline analysis.
- Add a user interface (e.g., buttons or a display) to allow real-time adjustment of the reference speed and adaptation gain.
- Extend the MRAC implementation to handle a second-order plant and design a suitable reference model for it. Compare the performance with the first-order case.
- Use two state variables (e.g., position and velocity) instead of just speed, and modify the MRAC algorithm accordingly. Analyze the stability and convergence of the new controller.
- Use state estimation techniques (e.g., Kalman filter, Luenberger observer) to improve the measurement of the plant states and analyze how it affects the performance of the MRAC controller.

## REFERENCE

- STMicroelectronics CubeF7 HAL, CMSIS2 and FreeRTOS libraries
