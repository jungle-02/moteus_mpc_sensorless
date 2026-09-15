# Sensorless PMSM Velocity Trajectory Tracking via Hybrid MPC-FOC and IASMO

[![MCU](https://img.shields.io/badge/MCU-STM32G474CEU6-blue.svg)](https://www.st.com/)
[![Driver](https://img.shields.io/badge/Hardware-moteus--n1-orange.svg)](https://mjbots.com)
[![License](https://img.shields.io/badge/License-Academic%20/ %20MIT-green.svg)](#license)
[![Python](https://img.shields.io/badge/GUI-PyQt5%20%7C%20ZeroMQ-yellow.svg)](#control-gui--monitoring)
[![Status](https://img.shields.io/badge/Status-Completed%20(Thesis%202026)-brightgreen.svg)](#thesis-citation)

This repository contains the full hardware setup, embedded firmware, mathematical formulations, and monitoring software for a high-performance **Sensorless Velocity Trajectory Tracking System** for **Permanent Magnet Synchronous Motors (PMSM)**. 

Developed as an Engineering Thesis at **Ho Chi Minh City University of Technology (HCMUT)** by **Lê Huỳnh Quốc Trung** under the guidance of **Dr. Nguyễn Vĩnh Hảo**.

---

## 📌 Table of Contents
- [Overview](#-overview)
- [Key Features](#-key-features)
- [System Architecture](#-system-architecture)
- [Control & Estimation Algorithms](#-control--estimation-algorithms)
  - [Hierarchical Cascade Control (MPC-FOC)](#1-hierarchical-cascade-control-mpc-foc)
  - [Improved Adaptive Sliding Mode Observer (IASMO)](#2-improved-adaptive-sliding-mode-observer-iasmo)
  - [Online RLS Parameter Identification & Kalman Torque Observer](#3-online-rls-parameter-identification--kalman-torque-observer)
- [Hardware Setup](#-hardware-setup)
- [Software & Firmware Structure](#-software--firmware-structure)
- [Control GUI & Monitoring](#-control-gui--monitoring)
- [Experimental Results & Benchmarks](#-experimental-results--benchmarks)
- [Getting Started](#-getting-started)
- [Thesis Citation](#-thesis-citation)
- [License & Acknowledgments](#-license--acknowledgments)

---

## 🛠 Overview

High-precision robotic arms, humanoid joints, and servo drives demand smooth velocity trajectory tracking under dynamic load changes. Traditional mechanical encoders increase system cost, size, and failure rates in harsh environments.

This project implements a **Cascaded Hybrid Control Framework** on the **moteus-n1 (STM32G474)** controller:
1. **Outer Velocity Loop (5 kHz)**: **Model Predictive Control (MPC)** with QP constraint optimization (Hildreth's algorithm) to plan optimal torque commands while respecting current/voltage limits.
2. **Inner Current Loop (20 kHz)**: **Field-Oriented Control (FOC)** with **Space Vector Pulse Width Modulation (SVPWM)** for fast, low-ripple current response ($i_d = 0$ MTPA).
3. **Hybrid State Estimation**: Uses magnetic encoder feedback (**AS5047P**) at low speeds ($\omega < 10\text{ rad/s}$) and smoothly transitions to an **Improved Adaptive Sliding Mode Observer (IASMO)** at medium-to-high speeds.
4. **Online Adaptability**: Integrates **Recursive Least Squares (RLS)** for real-time motor parameter tracking ($R_s, L_s, \phi_m$) and a **Kalman Filter** for load torque estimation.

---

## ✨ Key Features

- **Cascaded Real-Time Architecture**: Multi-rate interrupt strategy (20 kHz FOC / 5 kHz MPC).
- **Chattering-Free IASMO**: Replaces standard discontinuous `sign()` functions with continuous `tanh()` and adaptive gain laws to eliminate chattering and phase lag.
- **QP Hildreth Optimization**: Offline matrix pre-computation ($H^{-1}, F, G$) combined with fast online Hildreth iterations for real-time MPC on Cortex-M4F.
- **Thermal & Saturation Adaptation**: Online RLS continuously updates stator resistance $R_s$ and magnet flux $\phi_m$ as motor temperature rises.
- **Zero-Crossing Robustness**: Smooth trajectory recovery when transitioning through zero velocity and acceleration reversals.
- **PyQt5 Real-Time Dashboard**: High-speed telemetry streaming over **FDCAN** at 50 Hz via **ZeroMQ**.

---

## 🏗 System Architecture

```
                       +--------------------------------------------------+
                       |               PyQt5 Control GUI                  |
                       +-----------------------+--------------------------+
                                               | FDCAN (50 Hz)
                                               v
+-----------------------------------------------------------------------------------+
| STM32G474CEU6 Microcontroller (moteus-n1)                                         |
|                                                                                   |
|  +-------------------+  i_q_ref   +-------------------+  v_dq  +---------------+ |
|  |   MPC Outer Loop  | ---------> |  FOC Current Loop | ------> |     SVPWM     | |
|  |     (5 kHz)       |            |     (20 kHz)      |         |   (20 kHz)    | |
|  +-------------------+            +-------------------+         +-------+-------+ |
|            ^                                ^                           |         |
|            | T_load_hat                     | theta_est, w_est          | PWM     |
|    +-------+-------+              +---------+---------+                 v         |
|    | Kalman Filter |              | Hybrid IASMO /  |      +--------------------+ |
|    | Torque Est.   |              | Encoder Switch  |      | DRV8353S Gate Drv  | |
|    +---------------+              +-----------------+      +----------+---------+ |
|            ^                                ^                         |           |
+------------|--------------------------------|-------------------------|-----------+
             | T_e                            | i_a, i_b, i_c           | 3-Phase AC
             +--------------------------------+                         v
                                                                 +--------------+
                                                                 | SPMSM Motor  |
                                                                 |   (MJ5208)   |
                                                                 +--------------+
```

---

## 📐 Control & Estimation Algorithms

### 1. Hierarchical Cascade Control (MPC-FOC)
* **Outer Loop (MPC)**:
  Discrete state space model:
  $$\begin{bmatrix} \omega_m(k+1) \\ \theta_m(k+1) \end{bmatrix} = \begin{bmatrix} 1 - \frac{T_s B_m}{J} & 0 \\ T_s & 1 \end{bmatrix} \begin{bmatrix} \omega_m(k) \\ \theta_m(k) \end{bmatrix} + \begin{bmatrix} \frac{T_s}{J} \\ 0 \end{bmatrix} (T_e(k) - T_m(k))$$
  Formulated as a Quadratic Programming (QP) problem:
  $$\min_{\Delta U} \frac{1}{2} \Delta U^T H \Delta U + f^T \Delta U \quad \text{s.t.} \quad M \Delta U \le N$$
  Solved online via **Hildreth's QP algorithm**.

* **Inner Loop (FOC)**:
  PI current controllers tuned via **pole-zero cancellation**:
  $$K_p = \frac{L_s}{4 \zeta^2 \tau_d}, \quad K_i = \frac{R_s}{L_s} K_p$$

### 2. Improved Adaptive Sliding Mode Observer (IASMO)
* **Integral Sliding Surface**:
  $$S_n = \tilde{i}_s + \chi \int_0^t \tilde{i}_s d\tau$$
* **Adaptive Continuous Gain**:
  $$H(S) = \tanh(aS), \quad k_i(t) = \begin{cases} K_0 |S_i| & |S_i| \neq 0 \\ K_1 & |S_i| \approx 0 \end{cases}$$
* **Internal PLL Phase Locking**:
  $$\frac{d\hat{e}_\alpha}{dt} = -\hat{\omega}_e \hat{e}_\beta - l \tilde{e}_\alpha, \quad \frac{d\hat{e}_\beta}{dt} = \hat{\omega}_e \hat{e}_\alpha - l \tilde{e}_\beta, \quad \frac{d\hat{omega}_e}{dt} = \tilde{e}_\alpha \hat{e}_\beta - \tilde{e}_\beta \hat{e}_\alpha$$

### 3. Online RLS Parameter Identification & Kalman Torque Observer
* **RLS**: Dynamically estimates $R_s$, $L_d$, $L_q$, and flux linkage $\lambda_m$ with forgetting factor and thermal clamping.
* **Kalman Filter**: Estimates load torque $T_L$ from $T_e$ and estimated speed, passing smoothed $T_{\text{load,hat}}$ to MPC for immediate disturbance feedforward compensation.

---

## 🔌 Hardware Setup

| Component | Model / Specification | Function / Role |
| :--- | :--- | :--- |
| **Microcontroller** | **STM32G474CEU6** (170 MHz Cortex-M4F, CORDIC/FMAC) | Embedded Control Execution |
| **Power Stage / Driver** | **moteus-n1** + **DRV8353S** (3-Phase Smart Gate Driver) | Inverter Driver & Shunt Current Sensing |
| **Motor** | **MJ5208** SPMSM (7 pole pairs, $24\text{V}$, $R_s = 0.055\Omega$, $L_s = 25.6\mu\text{H}$) | Actuator / Target Plant |
| **Position Sensor** | **AS5047P** (14-bit Magnetic Encoder, SPI @ 10 MHz) | Low-speed / Calibration Feedback |
| **Communication** | **FDCAN Interface** (USB-to-CAN converter) | Telemetry & Command Streaming |

---

## 💻 Software & Firmware Structure

```
├── firmware/
│   ├── Core/
│   │   ├── Src/
│   │   │   ├── main.c              # Main loop (5 kHz MPC & Kalman trigger)
│   │   │   ├── foc_svpwm.c         # 20 kHz Timer 1 ISR for FOC & SVPWM
│   │   │   ├── iasmo_observer.c    # IASMO sliding mode observer implementation
│   │   │   ├── mpc_hildreth.c      # Hildreth QP solver & offline matrices
│   │   │   ├── rls_ident.c         # Online parameter identification
│   │   │   └── drv8353s.c          # SPI driver configuration
│   │   └── Inc/
│   │       └── motor_config.h      # Motor constants & loop tuning parameters
├── gui/
│   ├── main_gui.py                 # PyQt5 dashboard entry point
│   ├── fdcan_worker.py             # ZeroMQ & FDCAN interface thread
│   └── dashboards/                 # Real-time plotting widgets
├── simulation/
│   └── pmsm_mpc_iasmo_sim.slx      # MATLAB/Simulink validation model
└── README.md                       # Repository Documentation
```

---

## 🎛 Control GUI & Monitoring

The desktop GUI built with **PyQt5** features a 4-tab live dashboard communicating with the hardware at **50 Hz**:
1. **Control States**: Command vs. Encoder vs. Sensorless speed & position.
2. **Tracking Errors**: Real-time error dynamics and variance convergence ($\text{Var} < 0.09\text{ (rad/s)}^2$).
3. **Torque & Current**: $i_d, i_q$ currents and electromagnetic torque $T_e$.
4. **Parameter Estimation**: Live thermal tracking of $R_s$, $\lambda_m$, and $L_s$.

---

## 📊 Experimental Results & Benchmarks

Experimental tests conducted on the **moteus-n1 (STM32G474)** hardware platform across various velocity trajectories:

| Trajectory Type | Command Range | Steady-State Error | Max Dynamic Error | Performance Summary |
| :--- | :--- | :--- | :--- | :--- |
| **Step Response** | $0 \to 30 \to -20 \to 10\text{ rad/s}$ | $\mathbf{< 0.0032\text{ rad/s}}$ | $< 1.7\%$ | Fast rise time, zero overshoot, immediate convergence. |
| **Sinusoidal Wave** | $\pm 25\text{ rad/s}$ ($0.1\text{ Hz}$) | $\mathbf{\pm 0.5\text{ rad/s}}$ ($\approx 2\%$) | $< 4.0\%$ | Smooth zero-crossing phase tracking, low chattering. |
| **Bell-Shaped Curve** | $0 \to 30\text{ rad/s}$ | $\mathbf{\pm 0.5\text{ rad/s}}$ ($\approx 1.67\%$) | $< 3.5\%$ | Smooth acceleration and deceleration response. |
| **Triangular Wave** | $\pm 15\text{ rad/s}$ | $\mathbf{\pm 0.5\text{ rad/s}}$ ($\approx 3.3\%$) | $< 10\%$ (peaks) | Robust handling during rapid acceleration reversals. |

### Quantitative Comparison: Traditional PID vs. Proposed MPC-IASMO

| Parameter / Metric | Traditional Fixed-Gain PID | Proposed MPC-IASMO System |
| :--- | :--- | :--- |
| **Peak Tracking Error** | $\approx \pm 6.0\text{ rad/s}$ ($24.0\%$) | **$\approx \pm 1.0\text{ rad/s}$ ($4.0\%$)** |
| **Average Steady-State Error**| $\approx \pm 3.0\text{ rad/s}$ | **$\approx \pm 0.5\text{ rad/s}$ ($2.0\%$)** |
| **Chattering / Ripple** | Severe, dense oscillations | **Controlled, smooth convergence** |
| **Error Variance** | Unstable / Non-convergent | **Converges to $< 0.09\text{ (rad/s)}^2$** |
| **Disturbance Recovery** | Slow, susceptible to load steps | **$< 0.2\text{s}$ recovery under $0.1\text{ Nm}$ step load** |

---

## 🚀 Getting Started

### Prerequisites
* **Toolchain**: STM32CubeIDE / Arm GNU Toolchain (`arm-none-eabi-gcc`).
* **Python Environment**: Python 3.9+ with `PyQt5`, `pyzmq`, `python-can`, `matplotlib`, `numpy`.

### Building Firmware
```bash
# Clone the repository
git clone https://github.com/username/pmsm-mpc-iasmo-sensorless.git
cd pmsm-mpc-iasmo-sensorless/firmware

# Build firmware using STM32CubeIDE CLI or Makefile
make -j4
```

### Launching the Control GUI
```bash
cd gui
pip install -r requirements.txt
python main_gui.py
```

---

## 📜 Thesis Citation

If you find this work useful in your research or projects, please cite:

```bibtex
@mastersthesis{le2026sensorless,
  author       = {Lê Huỳnh Quốc Trung},
  title        = {Sensorless PMSM Velocity Trajectory Tracking via Hybrid MPC-FOC and IASMO},
  school       = {Ho Chi Minh City University of Technology (HCMUT)},
  year         = {2026},
  month        = {May},
  advisor      = {TS. Nguyễn Vĩnh Hảo},
  address      = {Ho Chi Minh City, Vietnam}
}
```

---

## 📄 License & Acknowledgments

* **License**: This project is licensed under the [MIT License](LICENSE).
* **Acknowledgments**: Special thanks to **Dr. Nguyễn Vĩnh Hảo** (Head of Automatic Control Department, HCMUT) for guidance, and colleagues Cao Thanh Vĩnh Hòa, Nguyễn Bùi Nguyên Khoa, Đặng Nhật Nam for experimental support.
