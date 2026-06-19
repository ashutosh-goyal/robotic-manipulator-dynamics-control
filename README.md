# Redundant Robots: RPRP and 4R Manipulator Dynamics and Simulation

## 📌 Project Overview
This repository contains the numerical implementations, kinematic simulations, and full dynamic models for multi-degree-of-freedom (DoF) redundant robotic manipulators. The core focus of this project is on the **4-DOF RPRP Manipulator** (2 revolute joints and 2 prismatic joints) and the **4R Serial Manipulator**. 

This project was developed by Ashutosh Goyal, Rajesh Kumar, and Ravi Dhorajia, under the guidance of Harish P.M. and Shail Jadav.

## 🧮 Mathematical Modeling & Dynamics
The dynamics for these robots were derived analytically using the **Lagrangian Formulation** ($\mathcal{L} = KE - PE$). The equations of motion are expressed in terms of the Lagrangian as follows:
$$\tau = \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q}$$

This is structured into standard matrix form calculating the Mass matrix, Coriolis terms, and Gravity vector.

### 1. The RPRP Manipulator
The RPRP robot utilizes joint variables $\theta_1(t), r_1(t), \theta_2(t), r_2(t)$. 
A major technical challenge addressed in this repository is **Prismatic Joint Constraining**. Without constraint, the prismatic links can travel infinitely unless stopped by a barrier. We devised two methods to constrain the length of the links mathematically:
* **Impulsive Force / Velocity Reflection:** Applying a strong impulsive force at the limiting conditions of the links so they refrain from moving in that direction, causing them to change their direction of motion opposite to how they were initially moving.
* **High-Stiffness Springs:** Adding virtual high-stiffness spring potential energy terms (e.g., $\frac{1}{2}k_1(l_{1_{max}}-r_1)^2$) to the Total Potential Energy ($PE$) equation to restrict the link within operational bounds.

### 2. The 4R Manipulator
A serial 4-DoF planar manipulator comprised entirely of revolute joints. The project computes the forward kinematics, constructs the highly coupled inertia matrix ($M_{ij}$), and resolves Coriolis/gravity force vectors.

---

## 📂 Repository File Structure

### 📄 Project Report
* **`RPRP_and_4R_Report.pdf`**: The official mathematical report detailing all Jacobian matrices, Forward Kinematics equations, and Lagrangian derivations.

### ⚙️ RPRP (4-DoF Hybrid) Scripts
* **`RPRP_dynamics_without_constraint.m`**: Simulates the standard unconstrained dynamics of the RPRP arm.
* **`RPRP_dynamics_flip_velocity.m`**: Implements the impulsive boundary constraint, instantly flipping the velocity vector upon reaching maximum extension.
* **`RPRP_dynamics_with_spring.m`**: Enforces physical boundaries using elastic limit buffers (spring potential field penalties).
* **`RPRP_dynamics_only_KE.m`**: Neutralizes gravity and potential energy to strictly analyze the inertial coupling and Kinetic Energy conservation.
* **`ode_solver_without_constraint.m`**: The core state-space differential equations solver used by the RPRP dynamic driver scripts.

### ⚙️ RP (2-DoF) Scripts
* **`RP_robot_simulation_only_kinematics.m`**: Forward kinematics and task-space trajectory visualization for an RP arm.
* **`RP_Dynamics_without_constraint.m`**: Free-fall forward dynamics of the RP arm using `ode23`.
* **`RP_Dynamics_with_constraint.m`**: Forward dynamics implementing unilateral geometric limits on the prismatic joint.

### ⚙️ Pure Revolute (3R & 4R) Scripts
* **`RRR_dynamics.m`**: Symbolic Lagrangian derivation for a 3-link planar revolute manipulator.
* **`RRRR_dynamics.m`**: Full symbolic multi-body formulation and energy equations for a 4-Degree-of-Freedom (4R) manipulator.

---

## 🚀 Usage & Execution
1. Ensure you have **MATLAB** installed (The Symbolic Math Toolbox is required for `RRR` and `RRRR` scripts).
2. Clone this repository to your local machine.
3. Open any driver script (e.g., `RPRP_dynamics_with_spring.m`) and run it in the MATLAB editor.
4. The scripts will output dynamic phase plots, robot animations, and **Total Energy (TE)** conservation graphs to mathematically validate the simulated models.
