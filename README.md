# baxter-arm-dynamic-model
Single Arm Dynamic Model of the Baxter Robot

These Matlab and Simulink files are for running a dynamic simulation of a single Rethink Robotics Baxter arm. 

These files are dependent on the matlab-rigid-body-viz package:
https://github.com/rpiRobotics/matlab-rigid-body-viz
and the general-robotics-toolbox package:
https://github.com/rpiRobotics/general-robotics-toolbox
Both packages should be added to the current path in Matlab while running. The required subfunctions should also be added to the path. The newest version of Matlab/Simulink may be required to run this simulation.

Dynamic parameters for the arm are taken from the Baxter URDF file. Joint friction is treated as viscous friction, and toy numbers are used. Future work should update these values to more realistic parameters. Arm dynamics are solved using the Newton-Euler formulation. A correction factor is added to the joint accelerations to account for a rigidly grasped load.

To run the demo, simply run the Baxter_dynamic_demo.m file. This demo simulates the robot following a predefined trajectory while rigidly grasping a load. An input-state feedack linearized controller designed for the Baxter arm under a rigid joint assumption (with a torque feedforward component) is used to control the robot.

To change the controller, simply swap out the controller block in the Simulink model. Likewise, the desired reference can be altered.

To change the grasped object geometric and inertial properties, see lines 49-56 in the Baxter_dynamic_demo.m file.

To remove the grasped object, enter the robot dynamic model block in the Simulink model and remove the joint correction factor block (this correction is added back to the joint accelerations). The Object and Contact models block should also be removed (top level of Simulink model).

Note that dynamic model matrices (M- Mass Inertia matrix, Cq - Coriolis and Centrifugal terms, and F - friction terms) are passed from the Robot Arm Dynamic Model block. These assume a rigid joint model, and are used with the controller. They can be editted or ignored as needed.
