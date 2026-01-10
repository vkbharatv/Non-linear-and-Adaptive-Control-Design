# Delay Control System

A Python implementation of a control system simulator of transfer functions that incorporates time delays, designed for analyzing and simulating control systems with delay elements.

## Overview

This project provides a `DelayControlSystem` class that simulates control systems with time delays. The simulator uses state-space representation and implements delay using a buffer mechanism to store past inputs.

## Features

- **State-space simulation**: Converts transfer functions to state-space form for numerical integration
- **Time delay modeling**: Implements delay using a circular buffer to store and retrieve past inputs
- **Flexible time stepping**: Configurable time step and simulation duration
- **Control loop simulation**: Supports closed-loop control with reference tracking
- **Performance metrics**: Calculates Integral Squared Error (ISE) for system performance analysis

## Requirements

```bash
pip install control numpy matplotlib scipy
```

## Dependencies

- `control`: Control systems library for transfer function and state-space operations
- `numpy`: Numerical computing library
- `matplotlib`: Plotting library for visualization
- `scipy`: Scientific computing library for signal processing

## Usage

### Basic Example

```python
import control as ctrl
from delay_control import DelayControlSystem

# Define transfer function
s = ctrl.TransferFunction.s
G = 1/(s+1)**2  # Second-order system

# Create delay system with 1 second delay
delay_time = 1.0
dt = 0.001
delay_system = DelayControlSystem(G, delay_time, dt=dt)

# Simulate step response
for i in range(1000):
    x, y, t = delay_system.step(u=1.0)
```

### Closed-Loop Control Example

The main script demonstrates a closed-loop control system with:
- Plant: $G(s) = \frac{e^{-\theta s}}{(s+1)^2}$
- Controller: $C(s) = 1 + \frac{0.1}{s}$ (PI controller)
- Output time delay $\theta$
- Square wave reference tracking

```python
s = ctrl.TransferFunction.s
G = 1/(s+1)**2
C = 1 + 0.1/s

delay_system = DelayControlSystem(G, delay_time=1, dt=0.001)
controller = DelayControlSystem(C, 0, dt=0.001)

# Simulation loop
for i in range(r.size):
    error = reference[i] - delay_system.y[-1]
    x, y, t = delay_system.step(u_c)
    _, u_c, _ = controller.step(error)
```

## Class: DelayControlSystem

### Constructor

```python
DelayControlSystem(tf, delay_time, dt=0.01, Total_time=10)
```

**Parameters:**
- `tf`: Transfer function (control.TransferFunction object)
- `delay_time`: Time delay in seconds
- `dt`: Time step for simulation (default: 0.01)
- `Total_time`: Total simulation time (default: 10)

### Methods

#### `step(u=1.0)`
Performs one simulation step with the given input.

**Parameters:**
- `u`: Input signal value

**Returns:**
- `x_next`: Next state vector
- `y_next`: Next output value
- `next_t`: Current time

#### `reset()`
Resets the system to initial conditions, clearing all stored data and buffers.

## How It Works

1. **Transfer Function to State-Space**: The system converts the transfer function to state-space form using `ctrl.tf2ss()`
2. **Delay Buffer**: Implements a circular buffer to store past inputs for the specified delay time
3. **Numerical Integration**: Uses Euler's method to integrate the state equations
4. **Output Calculation**: Computes output using $y = Cx + Du$

## Performance Metrics

The system calculates **Integral Squared Error (ISE)** as:

$$\text{ISE} = \int_0^T e(t)^2 \, dt$$

where $e(t) = r(t) - y(t)$ is the tracking error.

## Example Output

The main script generates plots showing:
- System output (blue)
- Tracking error (red dashed)
- Reference signal (green dashed)
- ISE value printed to console

## Applications

- Control system analysis with delays
- Performance evaluation of control algorithms
- Reference tracking studies
- Delay compensation research
- Control education and learning

## Notes

- The delay is implemented using a discrete buffer, so the actual delay is rounded to the nearest multiple of `dt`
- For systems without delay, set `delay_time=0`
- Smaller time steps (`dt`) provide more accurate simulations but increase computation time

## License

This project is provided as-is for educational and research purposes.

## Author

NLACD_T Course Project
