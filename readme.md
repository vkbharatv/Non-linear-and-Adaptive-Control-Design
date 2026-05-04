# Nonlinear and Adaptive Control Design

This repository collects course and research exercises on nonlinear control, adaptive control, and embedded control implementation. The workspace mixes Jupyter notebooks, Python scripts, Arduino code, MATLAB assets, and an STM32 FreeRTOS project.

The material progresses from transfer-function and delay-system studies to Model Reference Adaptive Control (MRAC), robust adaptive control, Sliding Mode Control (SMC), and real-time embedded deployment.

## Repository Overview

### Core topics

- Linear and delayed plant modeling
- PID-based control studies
- Direct and indirect MRAC
- Robust MRAC under uncertainty
- Sliding Mode Control
- Embedded motor control on Arduino and STM32

### Folder guide

| Folder | Description |
| --- | --- |
| `NLACD_T1` | Introductory transfer-function creation and Delay-system modeling. |
| `NLACD_T2` | Delay-system simulation and understanding the effects of time delays. |
| `NLACD_T3` | PID controller design and tuning for delayed systems. |
| `NLACD_T4` | Time-delay plant simulation and control analysis notebook. |
| `NLACD_T5` | Control loop design with Arduino for DC motor control. |
| `NLACD_T6_DirectMRAC` | Direct MRAC simulation with MATLAB and Python notebook. |
| `NLACD_T7_IndirectMRAC` | Indirect MRAC simulation in Python. |
| `NLACD_T8_DirectMRAC_STM32_rtos` | STM32F767 + FreeRTOS direct MRAC motor-control project. |
| `NLACD_T9_RobustMRAC` | Robust MRAC simulation for uncertain systems in Python. |
| `NLACD_T10_SMC` | Sliding Mode Control notebook. |

## Requirements

The Python examples use the dependencies listed in `requirements.txt`:

- `controlsim`
- `control`
- `matplotlib`
- `numpy`
- `scipy`

Install them with:

```bash
pip install -r requirements.txt
```

## Getting Started

### Python and notebooks

1. Create and activate a Python environment.
2. Install the packages from `requirements.txt`.
3. Open any notebook in the corresponding task folder and run the cells.
4. Run standalone scripts directly when provided, for example:

```bash
python NLACD_T7_IndirectMRAC/IndirectMRAC_Sim.py
python NLACD_T9_RobustMRAC/robustMrac.py
```

### Embedded targets

- `NLACD_T5` targets Arduino hardware and includes an `.ino` sketch with supporting C++ code.
- `NLACD_T8_DirectMRAC_STM32_rtos` targets STM32F767 hardware and can be built with STM32CubeIDE or CMake presets.

Build the STM32 project with VScode extenssion for STM32 microcontrollers or STM32CubeIDE, ensuring you have the appropriate toolchain and board support packages installed.

## Notes

- Some folders are notebook-first and are intended for interactive study rather than packaged execution.
- MATLAB and Simulink assets are included in `NLACD_T6_DirectMRAC` for model-based analysis.
- Generated build artifacts are present in a few folders and are kept alongside the source material.

## Author

**Author:** Dr. Bharat Verma  
**Note:** Assistant Professor, The LNMIIT, Jaipur, India  
**ORCID:** [https://orcid.org/0000-0001-7600-7872](https://orcid.org/0000-0001-7600-7872)  
**GitHub:** [https://github.com/vkbharatv](https://github.com/vkbharatv)
