# 2D Robotic Arm Simulation

An interactive 2D robotic arm simulation developed for CMPUT 340 at the University of Alberta. The project explores forward and inverse kinematics, Jacobian matrices, and numerical optimization methods for controlling a two-link robotic arm.

The original implementation was written in MATLAB. An interactive web visualization is intended to make the simulation easier to explore by allowing users to select target positions, observe the arm's movement, and compare inverse kinematics methods.

## Features

- **Forward kinematics:** Calculate the end-effector position from link lengths and joint angles.
- **Analytical Jacobian:** Derive the Jacobian matrix to describe how changes in joint angles affect the end-effector position.
- **Finite-difference Jacobian:** Approximate the Jacobian numerically using central finite differences.
- **Inverse kinematics:** Calculate joint angles for a desired end-effector position using Newton's method or Broyden's method.
- **Interactive visualization:** Explore the relationship between target positions, joint configurations, and arm movement through a graphical interface.
- **Numerical method comparison:** Examine the differences between using an explicitly calculated Jacobian and updating a Jacobian approximation iteratively.

## Interactive Demo

**Live demo:** [Open the robotic arm simulation](https://YOUR-USERNAME.github.io/YOUR-REPOSITORY/)

The visualization is designed to let users select a target position and observe the arm's configuration as the inverse kinematics algorithm attempts to reach it.

Replace the placeholder URL with the GitHub Pages URL after enabling deployment for the repository.

## Mathematical Background

### Forward Kinematics

For a two-link planar robotic arm with link lengths \(l_1, l_2\) and joint angles \(\theta_1, \theta_2\), the end-effector position is

\[
x=l_1\cos\theta_1+l_2\cos(\theta_1+\theta_2)
\]

\[
y=l_1\sin\theta_1+l_2\sin(\theta_1+\theta_2)
\]

These equations map the joint configuration to a position in Cartesian space.

### Jacobian Matrix

The analytical Jacobian describes the local relationship between joint-angle changes and end-effector displacement:

\[
J(\theta)=
\begin{bmatrix}
\frac{\partial x}{\partial\theta_1} &
\frac{\partial x}{\partial\theta_2}\\
\frac{\partial y}{\partial\theta_1} &
\frac{\partial y}{\partial\theta_2}
\end{bmatrix}
\]

The project also implements a central finite-difference approximation to estimate the Jacobian numerically. This provides a way to examine the accuracy and computational trade-offs of analytical and numerical differentiation.

### Inverse Kinematics

The simulation implements two iterative approaches:

- **Newton's method:** Uses the analytical Jacobian to calculate joint-angle updates.
- **Broyden's method:** Updates an approximation of the Jacobian using successive changes in the joint angles and position error, avoiding a complete Jacobian recalculation at every iteration.

Both methods attempt to minimize the difference between the current end-effector position and a desired target, subject to the iteration limit and convergence tolerance.

## MATLAB Implementation

The original MATLAB implementation includes the following functions:

| Function | Description |
|---|---|
| `evalRobot2D` | Calculates the end-effector position and analytical Jacobian. |
| `fdJacob2D` | Approximates the Jacobian using central finite differences. |
| `invKin2D` | Solves the inverse kinematics problem using Newton's or Broyden's method. |
| `plotRobot2D` | Visualizes the robot's links and joints. |
| `plotSeg` | Draws an individual link segment and its local axes. |
| `mycircle` | Draws circular markers for the joints. |

## Running the MATLAB Version

1. Install MATLAB.
2. Add the MATLAB source files to your working directory.
3. Configure the link lengths, iteration limit, and inverse kinematics method.
4. Run the simulation script.

The original example uses two links of length `0.5`, a randomly initialized joint configuration, and interactive target selection through MATLAB's `ginput()` function.

## Limitations

The simulation models a two-link planar robotic arm and does not account for physical dynamics, actuator constraints, or collision detection. Inverse kinematics convergence depends on the target position, initial joint configuration, numerical conditioning, and iteration limit.

## Project Context

Developed as part of CMPUT 340 at the University of Alberta (September–December 2024), applying linear algebra and numerical optimization to a robotics problem.
