# README

CRANEbot Simulation and Control

## Overview

This project focuses on the simulation and control of robotic systems, particularly the CRANEbot. It includes various datasets, function libraries, parameter files, and scripts for different aspects of robotic control and simulation.

## Folder Structure

```
data/
	cranebot/
	T-Probe/
docs/
functions/
	control_functions/
	kinematics_functions/
	positioning_functions/
	screw_theory_functions/
	screw_theory_symbolic_functions/
	skeleton_functions/
	trajectory_generation_functions/
parameters/
	computationComInertias.m
	cranebot_parameters.m
	cranebotPilzParameters.m
	parameters_3R.m
README.md
scripts/
	experimental_data_analysis/
	identification_validation/
	model_free_analysis/
	model_predictive_control/
	simscape_simulations/
	symbolic_model_generation/
```

## Folder Descriptions

### data
Contains subdirectories for different datasets used in the project.

- `cranebot/`: Newer data taken on the cranebot.
- `T-Probe/`: Data related to the data captured with a Leica T-Probe.

### data
Contains other useful documents

- `DynamicsPlanar2R`: explanation of usage of the functions for dynamics generations. Planar 2R example 
- `ACADOS_MATLAB_Workflow`: Files describing folder related to MPC control

### functions
Contains various function libraries organized by their functionality.

- `control_functions/`: Functions related to control algorithms.
- `kinematics_functions/`: Functions for kinematic calculations.
- `positioning_functions/`: Functions for positioning (transport) tasks.
- `screw_theory_functions/`: Functions for generation of dynamics based on screw theory.
- `screw_theory_symbolic_functions/`: Functions for symbolic generation of dynamics based on screw theory.
- `skeleton_functions/`: Functions related to the skeleton algorithm (anti-self-collision algorithm).
- `trajectory_generation_functions/`: Functions for generating trajectories.

### parameters
Contains parameter files used in simulations and calculations.

- `computationComInertias.m`: Script for computing inertias.
- `cranebot_parameters.m`: Parameters specific to the CRANEbot.
- `cranebotPilzParameters.m`: Parameters for the CRANEbot with Pilz model.
- `parameters_3R.m`: Parameters for a 3R robot model.

### scripts
Contains various scripts organized by their purpose.

- `experimental_data_analysis/`: Scripts for analyzing experimental data.
- `identification_validation/`: Scripts for system identification and validation.
- `model_free_analysis/`: Scripts for model-free analysis.
- `model_predictive_control/`: Scripts for model predictive control.
- `simscape_simulations/`: Scripts for Simscape simulations.
  - `arms_motion/`: Simulations related to arm motion.
  - `crane_motion/`: Simulations related to crane motion.
  - `manipulation/`: Simulations related to manipulation tasks.
  - `positioning/`: Simulations related to positioning tasks.
  - `skeleton_algorithm/`: Simulations related to the skeleton algorithm.
  - `uncontrolled/`: Simulations without control.
- `symbolic_model_generation/`: Scripts for generating symbolic models.
  - `cranebot/`: Symbolic models for the CRANEbot.
  - `planar_dual_arm/`: Symbolic models for planar dual-arm robots.

## Getting Started

### Prerequisites

- MATLAB 2020b or higher (for running `.m` scripts)
- Simulink (for Simscape simulations)
- Simscape
- Simscape Multibody

### Installation

1. Clone the repository:
   ```sh
   git clone https://gitlab.cern.ch/mro/robotics/analytic-and-numeric-analysis/numeric-robot-simulation/cranebot.git
   ```
2. Navigate to the project directory:
   ```sh
   cd cranebot
   ```


## Acknowledgements

- [MATLAB](https://www.mathworks.com/products/matlab.html)
- [Simulink](https://www.mathworks.com/products/simulink.html)
- [Simscape](https://www.mathworks.com/products/simscape.html)
- [Simscape Multibody](https://www.mathworks.com/products/simscape-multibody.html)

# References
- Master Thesis Giancarlo D'Ago 
- Master Thesis PhD Thesis
- Presentations Giancarlo D'Ago (student's coffee and others)
- Master Thesis Michele Avagnale


## Contact

For any questions or feedback, please contact [giancarlo.dago.97@gmail.com](mailto:giancarlo.dago.97@gmail.com).

---
