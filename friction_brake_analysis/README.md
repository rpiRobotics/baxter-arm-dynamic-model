
# Friction Brake Analysis

This folder contains files related to the friction brake analysis for the Baxter arm. To determine how the arm will react when friction brakes are applied, a dynamic simulation of the arm under friction braking is run. The resulting trajectory after braking, as well as deviation from the pose at the point where brakes are applied, is used to investigate the behavior. 

This code is reliant on the subfunctions folder in the baxter_arm_dynamic_model repository, as well as the general_robotics_toolbox:
https://github.com/rpiRobotics/general-robotics-toolbox

Currently, this analysis is only for the rigid joint Baxter model transporting an air bearing load. Future work will update with the flexible joint model. 

# File Descriptions:
# Scripts:
trap_gen_check.m: script used to generate and test trapezoidal velocity motion profile for feasibility
friction_braking_analysis.m: run the friction brake simulation; plot results

# .mat files:
.mat files defining the path and trajectory are: ab_path_long.mat and ab_trap_traj.mat. 

# Other Files:
Baxter_brake_sim.slx: simulink simulation for applied brakes. Run from main script friction_braking_analysis.m
