# Intelligent Mobile Robot Navigation using Hybrid Fuzzy-GA

This repository contains the simulation and report for an autonomous mobile robot navigation system developed using a **Hybrid Fuzzy-Genetic Algorithm (GA)** architecture. The system enables a point-mass robot to navigate from a starting point to a designated goal while avoiding static obstacles in 2D environments.

## Project Overview
- **Objective:** Achieve reliable target acquisition with collision-free trajectories in both simple and complex, high-density obstacle environments.
- **Methodology:** Integration of a Fuzzy Logic Controller (FLC) for real-time navigation decision-making, combined with a Genetic Algorithm (GA) to automatically optimize the sensor input membership functions.
- **Tech Stack:** Python (NumPy, Matplotlib, Pygame, Scikit-fuzzy).

## System Architecture
The robot operates based on a closed **perception-decision-action** control loop:
1. **Perception (Sensors):** Four virtual, noise-free sensors measuring obstacle distances (Front, Left, Right) and the relative goal bearing angle.
2. **Decision (Controller):** A Mamdani-type Fuzzy Inference Engine that uses intuitive human-like "IF-THEN" linguistic rules to determine steering and velocity outputs.
3. **Action & Optimization:** A Genetic Algorithm that evaluates candidate fuzzy parameters against a multi-objective fitness function (balancing target arrival, collision penalties, trajectory directness, and time steps) to eliminate the need for tedious manual tuning.

## Performance Results
| Metric | Baseline Fuzzy Controller | Hybrid GA-Fuzzy Controller |
| :--- | :--- | :--- |
| **Simple Map** | Successfully reaches goal with no collisions. | Successfully reaches goal with an optimized, efficient path. |
| **Complex Map** | Poor performance; struggles with sharp turns and high-density obstacles (high collisions). | Near-perfect navigation; successfully avoids local minima and tight spaces. |

## Group Members (IIUM - Section 1)
* **Syed Mohamad Syafiee Bin Syed Tajudin** (2318483)
* **Muhamad Harith Haikal Bin Zulkifli** (2319963)
* **Muhammad Farqi Sarhidi Bin Abdul Aziz** (2316463)

## Course Details
- **Subject:** Computational Intelligence (MCTA 3371)
- **Lecturers:** Dr. Azhar Bin Mohd Ibrahim & Dr. Amir Akramin Bin Shafie
- **Date of Submission:** 7 June 2026 (Sunday)
