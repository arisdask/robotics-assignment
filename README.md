# Robotics Assignment

MATLAB implementation of the 2025 Robotics assignment (ECE AUTh), including full kinematic analysis, trajectory generation, and simulation results.

## What is implemented

### Part A — Door/Handle Motion Planning
- Builds a smooth two-stage door/handle motion over the required time window.
- Computes handle position, orientation, velocity, and acceleration through the trajectory.
- Produces plots and animations to verify the planned motion.

### Part B — UR10 Control to Execute the Motion
- Uses a UR10 model to follow the motion planned in Part A.
- Solves for joint commands over time with numerical methods.
- Compares solver behavior, plots joint trajectories, and validates tracking with final visualizations.

## Project layout
- `main.m`: main executable script for both Part A and Part B.
- `src/`: core computation utilities.
- `utils/`: plotting and animation helpers.
- `robot_model/ur10robot.m`: UR10 model definition.
- `plotsPartA/`, `plotsPartB/`: exported figures used in the report.

## How to run
1. Open the project in MATLAB.
2. Ensure Robotics Toolbox / Spatial Math dependencies used by `ur10robot`, `SerialLink`, and `UnitQuaternion` are available.
3. Run:
	- `main.m` (executes both parts and generates plots/animations)
4. Optional:
	- Set `debugMode = 0` in `main.m` to skip pause prompts between animation sections.

## Notes
- The report PDF (`arisdask_Robotics_Project.pdf`) includes the complete analysis and results.
