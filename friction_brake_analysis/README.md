
# Friction Brake Analysis

This folder contains files related to the friction brake analysis for the Baxter arm. To determine how the arm will react when friction brakes are applied, a dynamic simulation of the arm under friction braking is run. The resulting trajectory after braking, as well as deviation from the pose at the point where brakes are applied, is used to investigate the behavior. Separate analysis scripts/simulink models are provided for the rigid and flexible joint robot models.

This code is reliant on the subfunctions folder in the baxter_arm_dynamic_model repository, as well as the general_robotics_toolbox:
https://github.com/rpiRobotics/general-robotics-toolbox


# File Descriptions:
# Scripts:
trap_gen_check.m: script used to generate and test trapezoidal velocity motion profile for feasibility (rigid joint model only)
friction_braking_analysis.m: run the friction brake simulation - rigid joint model; plot results
friction_braking_analysis_flex.m: run the friction brake simulation - flexible joint model; plot results


# .mat files:
.mat files defining the path and trajectory for rigid joint model are: ab_path_long.mat and ab_trap_traj.mat. 
.mat files defining the path and trajectory for the flexible joint model are: ab_path_long.mat, bs_traj.mat, bs_traj_js.mat. Note that the flexible joint trajectories are generated using a bounded snap motion profile algorithm. 

# Other Files:
Baxter_brake_sim.slx: simulink simulation for applied brakes - rigid joint model. Run from main script friction_braking_analysis.m
Baxter_brake_flex_sim.slx: simulink simulation for applied brakes - flexible joint model. Run from main script friction_braking_analysis_flex.m
