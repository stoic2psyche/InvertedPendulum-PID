# 🌀 Inverted Pendulum Control using PID in MATLAB

## 📌 Project Overview

This project demonstrates the **stabilization of an inverted pendulum on a cart** using a **PID controller** designed and simulated in MATLAB. The inverted pendulum is a classical problem in control systems, widely studied due to its nonlinear and unstable nature. The objective is to design a PID controller that stabilizes the pendulum in the upright position while also ensuring minimal oscillations in the cart position.

## 🎯 Objectives

- Model the dynamics of an inverted pendulum-cart system  
- Derive the transfer function from force input to cart position  
- Design and tune a PID controller to stabilize the system  
- Simulate and visualize the step response of the closed-loop system  
- Analyze system performance and stability  

## ⚙️ System Parameters

| Parameter | Description                          | Value        |
|----------:|--------------------------------------|-------------:|
| `M`       | Mass of the cart                     | 0.5 kg       |
| `m`       | Mass of the pendulum                 | 0.2 kg       |
| `b`       | Coefficient of friction              | 0.1 N/m/sec  |
| `I`       | Moment of inertia of the pendulum    | 0.006 kg·m²  |
| `g`       | Acceleration due to gravity          | 9.8 m/s²     |
| `l`       | Distance to pendulum's center of mass| 0.3 m        |

The transfer function is derived based on Newtonian mechanics, and linearization is applied around the upright equilibrium.

## 🧠 Control Strategy

A **PID controller** is designed with the following structure:
C(s) = Kp + Ki/s + Kd*s


The controller gains are tuned to achieve desired time-domain specifications such as:
- Fast settling time
- Minimal overshoot
- Stable response

Example tuned values:
- `Kp = 339`
- `Ki = 758`
- `Kd = 37.9`

## 📊 Simulation Steps

1. Define physical parameters and derive the plant transfer function  
2. Use `pidTuner` in MATLAB to interactively tune the PID controller  
3. Implement the closed-loop system using the `feedback()` function  
4. Simulate the step response using `step()` to analyze system behavior  

## 📈 Results

The tuned controller successfully stabilizes the inverted pendulum. The step response plot shows a controlled system with fast rise time and acceptable overshoot, verifying the effectiveness of PID control for this unstable system.

## 🛠️ Tools Used

- **MATLAB** (Control System Toolbox)
- `tf`, `feedback`, `step`, and `pidTuner` functions





