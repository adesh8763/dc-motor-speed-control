# DC Motor Speed Control using PID

This project demonstrates modeling and speed control of a DC motor using classical control techniques.  
The motor dynamics are modeled and a PID controller is designed to regulate motor speed.

## Motor Parameters

| Parameter | Value |
|----------|------|
| Moment of Inertia (J) | 0.01 kg·m² |
| Viscous Friction (b) | 0.1 N·m·s |
| Torque Constant (Kt) | 0.01 Nm/A |
| Back EMF Constant (Ke) | 0.01 V·s/rad |
| Resistance (R) | 1 Ω |
| Inductance (L) | 0.5 H |

## Mathematical Model

The DC motor transfer function between input voltage and angular velocity is:

ω(s) / V(s)

derived from the electrical and mechanical equations of the motor.

## Control Objective

Design a controller to:

- achieve stable speed control
- minimize steady-state error
- reduce rise time
- limit overshoot

## Controller Design

A **PID controller** is designed and tuned in MATLAB.

Controller structure:

u(t) = Kp e(t) + Ki ∫e(t)dt + Kd de/dt

## Simulation

Simulation was performed using:

- MATLAB
- Simulink

Results show improved speed response with the PID controller.

## Results

- Reduced steady state error
- Faster rise time
- Stable closed-loop behavior


## Author

Adesh Prasad Dash

Mechanical Engineering Student  
VIT

## Future Work

- State-space control
- LQR controller
- Real hardware implementation with Arduino/ESP32
