% Baxter Arm Simulation
% David Carabis
% RPI

% This dynamic model simulation is of the Baxter Arm using the
% Newton-Euler algorithmic formulation. 
%
% Dynamic and kinematic parameters are based off of the Baxter URDF file
% from Rethink Robotics: https://github.com/RethinkRobotics/baxter_common/blob/master/baxter_description/urdf/baxter.urdf
%
% Toy viscous friction values are chosen for the joint friction models.
%
% Each joint is modeled as a flexible joint.
%
% List of simulation output variables:
% t: simulation time
% qddot: joint acceleration 
% qdot: joint velocity
% q: joint position
% V0T: spatial velocity of end effector
% P0T: end effector position 
% R0T: end effector orientation (rotation matrix)
% f_tip: spatial force applied to object/environment
% f_c: spatial force experienced by object at object COM frame C
% f_base: spatial force at the manipulator arm base
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
close all
clc

%% Simulation Setup
T = 10; % Total Simulation Time
animate_robot = 1; % 1 for animation, 0 for no animation
% NOTE: Animation is not real-time (nor constant time). Due to
% variable-step solver, the time-steps used in the animation may not be
% constant. Resample the data to generate a movie showing realistic motion
% over time. Use this built-in animation as a sanity check while running
% simulations.

x = [1;0;0]; y = [0;1;0]; z = [0;0;1]; zerovec = [0;0;0]; 
hg = -x; g = 0; % gravity vector and magnitude (set g = 0 for no gravity)

% Initial Conditions
V0 = [0;0;0;0;0;0];
A0 = [0;0;0;g*hg];

%% Mass Properties - Set properties of grasped mass
% Cube
m = 1; % mass (kg)
l = 0.25; % length (m)
Ic = m*l^2/6*eye(3); % Inertia tensor
ric_tip = [0.5*l;0;0.25*l]; % vector from contact point to center of object mass, in the end effector frame
Mc = [[Ic zeros(3)];[zeros(3) m*eye(3)] ]; % mass-inertia matrix
Mc_inv = inv(Mc); % inverse of mass-inertia matrix

%% Controller Parameters - Edit this section per your control choice

% Load precomputed trajectory - put in timeseries format for Simulink
load traj_baxter_demo.mat
q_star = [t',q'];
qdot_star = [t',qdot'];
qddot_star = [t',qddot'];
tau_star = [t',tau'];
clear t qddot qdot q tau

gain_multiplier = 5;

% Initial Joint Angles
q0 = [0;-30;0;15;15;15;20]*pi/180;

% Build Robot Constants and Solve for Initial Pose
[robot_const, robot_structure] = defineBaxterSingleArmFlex();
N = robot_const.mprops.N; % Specify Gear Ratios (if any)
[R0T0, P0T0] = fwdkin(robot_const.kin,q0);
% Assume motor initial conditions are such that springs are not loaded:
qm0 = q0.*N; 

%% Run Simulation
tic % simulation timer
sim('Baxter_dynamic_demo_sim.slx')
display('Simulation Time:')
toc
%% Visualize

% Creating Full Robot
figure(1); clf;
h_baxter = createCombinedRobot(robot_const, robot_structure);
h_object = drawCube(object_pos_signal(1,:),l,object_rot_signal(:,:,1));
axis equal
axis([-0.5 1.5 -1 1 -1.5 1.5]);
view([-160 3]);
xlabel('x')
ylabel('y')
zlabel('z')
grid on

% Pre-allocate angle structure
Theta = get_angle_structure(h_baxter);

% Step through time sequence and update robot at each time instance
counter = 0;
if animate_robot > 0
    for k=1:80:length(t)
        Theta.state = [q(k,1) q(k,2) q(k,3) q(k,4) q(k,5) q(k,6) q(k,7)];
        h_baxter = updateRobot(Theta,h_baxter);
        h_object = updateCube(object_pos_signal(k,:),l,object_rot_signal(:,:,k),h_object);
        drawnow;
    end
end

%% Plot Desired and Actual Joint Angles
figure(2)
for k = 1:7
    if k==7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(t,q(:,k),'b','LineWidth',2)
    hold on
    plot(q_star(:,1),q_star(:,k+1),'r--','LineWidth',2)
    xlabel('Time (s)')
    if (k==1)||(k==4)||(k==7)
        ylabel('Joint Position (rad)')
    end
    if k==1
        legend({'Position','Desired Position'})
    end
    title(['q',num2str(k)])
    
end
