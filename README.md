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


# Redundant Robots: RPRP and 4R Manipulator Dynamics and Simulation

## 📌 Project Overview
This repository contains the numerical implementations, kinematic simulations, and full dynamic models for multi-degree-of-freedom (DoF) redundant robotic manipulators. The core focus of this project is on the **4-DOF RPRP Manipulator** (2 revolute joints and 2 prismatic joints), the **4R Serial Manipulator**, and the **3R Manipulator**. 

This project was developed by Ashutosh Goyal, Rajesh Kumar, and Ravi Dhorajia, under the guidance of Harish P.M. and Shail Jadav.

---

## 🎥 Visual Demonstrations & Results

### 1. Complex Trajectory Tracking (3R Manipulator)
The inverse kinematics and trajectory planning of the 3R redundant manipulator tracking a continuous Lissajous (figure-8 / infinity) curve. The redundancy is resolved using the Moore-Penrose pseudo-inverse Jacobian.
![3R Infinity Trajectory](./media/3R_infinity.gif) *(Place your video/gif here)*

### 2. Prismatic Joint Constraints: The Challenge
A major technical challenge in simulating the RPRP robot is **Prismatic Joint Constraining**. Without constraints, prismatic links travel infinitely. We tested two methods to solve this:

**Attempt 1: Velocity Reflection / Flipping (Failed)**
Our initial approach was to apply an impulsive boundary condition: the moment the link hits its maximum limit, we instantaneously invert its velocity ($\dot{r} \to -\dot{r}$).
* **Result:** The simulation gets "stuck" and freezes at the boundary.
* **Why it fails (Numerical Chattering):** MATLAB's adaptive step-size solvers (like `ode45` or `ode23`) rely on continuous derivatives. An instantaneous flip in velocity creates a severe mathematical discontinuity. When the link hits the boundary, the solver flips the velocity. However, due to continuous forces (like gravity or Coriolis) pushing it back, and finite numerical precision, the solver steps slightly past the boundary again in the next micro-step. This causes the velocity to flip back and forth infinitely in infinitesimally small time steps—a phenomenon known as **Zeno's paradox or numerical chattering**—forcing the solver to reduce its step size to zero and freeze.

![Velocity Flip Stuck](./media/RPRP_velocity_stuck.gif) *(Place your failed velocity flip video here)*

**Attempt 2: High-Stiffness Potential Springs (Success)**
To solve the discontinuity issue, we introduced virtual high-stiffness spring potential energy terms (e.g., $PE_{spring} = \frac{1}{2}k(l_{max}-r)^2$) only when the link exceeds its bounds. 
* **Result:** The simulation works perfectly. The stiff spring provides a highly repulsive but **continuous** force that the ODE solver can smoothly integrate, gracefully decelerating and reversing the link without mathematical singularities.

![Spring Constraint Success](./media/RPRP_spring_success.gif) *(Place your successful spring video here)*

### 3. Validation via Energy Conservation
To prove the physical and mathematical validity of our Lagrangian models, we track the system's energies over time without external torques. As expected in a conservative system, the Kinetic Energy (KE) and Potential Energy (PE) heavily fluctuate, but the **Total Energy (TE = KE + PE) remains perfectly constant**.
![Energy Conservation Graph](./media/energy_graph.png) *(Place your TE/KE/PE graph here)*

---

## 🧮 Mathematical Modeling & Dynamics
The dynamics for these robots were derived analytically using the **Lagrangian Formulation** ($\mathcal{L} = KE - PE$). The equations of motion are expressed in terms of the Lagrangian as follows:
$$\tau = \frac{d}{dt} \left( \frac{\partial \mathcal{L}}{\partial \dot{q}} \right) - \frac{\partial \mathcal{L}}{\partial q}$$

This is structured into the standard matrix form, calculating the highly coupled Mass/Inertia matrix ($M_{ij}$), Coriolis/Centrifugal terms ($C$), and Gravity vector ($G$).

---

## 📂 Repository File Structure

To keep the repository clean, all MATLAB scripts are located in the `src/` directory, and all mathematical derivations are in the root report.

```text
📦 robotic-manipulator-dynamics-control
 ┣ 📂 src (MATLAB Source Code)
 ┃ ┣ 📜 RPRP_dynamics_with_spring.m      # Successful continuous boundary constraint
 ┃ ┣ 📜 RPRP_dynamics_flip_velocity.m    # Impulsive boundary constraint (Chattering example)
 ┃ ┣ 📜 RPRP_dynamics_without_constraint.m
 ┃ ┣ 📜 RPRP_dynamics_only_KE.m
 ┃ ┣ 📜 ode_solver_without_constraint.m  # Core state-space ODE solver
 ┃ ┣ 📜 RP_robot_simulation_only_kinematics.m
 ┃ ┣ 📜 RP_Dynamics_without_constraint.m
 ┃ ┣ 📜 RP_Dynamics_with_constraint.m
 ┃ ┣ 📜 RRR_dynamics.m
 ┃ ┣ 📜 RRRR_dynamics.m
 ┃ ┗ 📜 Infinity_3R.m                    # 3R trajectory tracking
 ┣ 📂 media (Visual Assets)
 ┃ ┣ 🖼️ energy_graph.png
 ┃ ┣ 🎥 3R_infinity.gif
 ┃ ┣ 🎥 RPRP_velocity_stuck.gif
 ┃ ┗ 🎥 RPRP_spring_success.gif
 ┣ 📄 RPRP_and_4R_Report.pdf             # Official mathematical report
 ┗ 📄 README.md
