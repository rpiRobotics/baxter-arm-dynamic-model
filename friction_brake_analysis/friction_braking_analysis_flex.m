% Friction Braking Analysis - Flexible Joint

% This script investigates the effects of applying friction brakes to each
% actuator joint. The robot model assumes flexible joints and a coulomb +
% viscous friction model for the brakes. A path and trajectory of the arm
% are assumed, and a starting point along this trajectory is chosen as the
% initial condition (when the brakes are applied). The simulation output
% provides the arm trajectory during breaking, which is plotted against the
% desired trajectory.

% Note that this code assumes the trajectory under consideration is
% feasible, and does not test trajectory feasibility against dynamic
% constraints

% Note that an assumed trajectory is not necessary for the analysis - this
% trajectory is only included as a visualization aid for determining
% deviation from the desired trajectory. Future work should address
% deviation from the path only (independent of the trajectory).

clear variables; close all; clc

%% Specify Friction Brake Model and Simulation Time
T = 3; % Sim time
Tc = [1;1;1;1;0.5;0.5;0.5]; % coulomb brake friction
Tb = 10*ones(7,1); % viscous brake friction

%% Load Path and Trajectories; Define Robot and Object 
load ab_path_long.mat % feasible air bearing path
load bs_traj.mat
lambda_traj = p_sym; lambda_dot_traj = vel_sym; lambda_ddot_traj = acc_sym;
lambda_jerk_traj = jerk_sym; lambda_snap_traj = snap_sym;
clear p_sym vel_sym acc_sym jerk_sym snap_sym
load bs_traj_js.mat % feasible air bearing trajectory - note this is path specific

% Uniqueness Check:
[~,ia,~] = unique(lambda_traj);
lambda_traj = lambda_traj(ia); 
lambda_dot_traj = lambda_dot_traj(ia);
lambda_ddot_traj = lambda_ddot_traj(ia);
lambda_jerk_traj = lambda_jerk_traj(ia);
lambda_snap_traj = lambda_snap_traj(ia);
t_traj = t_traj(ia);
q = q(:,ia); qdot = qdot(:,ia); qddot = qddot(:,ia);
qjerk = qjerk(:,ia); qsnap = qsnap(:,ia);

% Define Baxter Robot
[robot_const, ~] = defineBaxterSingleArmFlex();
A0 = zeros(6,1); % base acceleration configuration - assumes no gravity
V0 = zeros(6,1); % no base motion (fixed base)

% Define grasped object geometry and mass properties:
% (Currently set for air bearing object)
m = 14.22; % mass (kg)
r = 0.4318/2; % radius
ht = 0.2794; % height
Ic = [(0.5*m*(3*r^2 + ht^2)) 0 0; 0 (0.5*m*(3*r^2 + ht^2)) 0; ...
    0 0 0.5*m*r^2]; % cylinder model
RTs = rot([0;0;1],-42*pi/180); % end effector to object center rotation matrix
PTs = RTs*[-0.28;0.35079;-0.06]; % end effector to object center vector
ric_tip = PTs;
Mc = [[Ic zeros(3)];[zeros(3) m*eye(3)] ]; % mass-inertia
Mc_inv = inv(Mc); % inverse of mass-inertia matrix

% Compute task space trajectory:
P0T = zeros(3,length(lambda_traj));
rpy = zeros(3,length(lambda_traj));
P0s = zeros(3,length(lambda_traj));
rpy_s = zeros(3,length(lambda_traj));

for k = 1:length(lambda_traj)
    [Rtemp, ptemp] = fwdkin(robot_const.kin, q(:,k));
    P0T(:,k) = ptemp;
    P0s(:,k) = ptemp + Rtemp*PTs;
    rpy_temp = rotm2eul(Rtemp,'XYZ');
    rpy(:,k) = [rpy_temp(1);rpy_temp(2);rpy_temp(3)];
    Rstemp = Rtemp*RTs;
    rpy_s_temp = rotm2eul(Rstemp,'XYZ');
    rpy_s(:,k) = [rpy_s_temp(1);rpy_s_temp(2);rpy_s_temp(3)];
end

%% Allow User to Specify Lambda Sample Point and Generate Initial Conditions
figure(1)
plot(lambda_traj,lambda_dot_traj,'b--','LineWidth',2)
xlabel('$\lambda$','Interpreter','Latex')
ylabel('$\dot{\lambda}$','Interpreter','Latex')

no_sample = 1;

while no_sample
    lambda_brk = input('Please specify lambda sample point:');
    if (lambda_brk>=0)&&(lambda_brk<=1)
        no_sample = 0;
    else
        disp('Sample must be between 0 and 1')
    end
end

close 1

lambda_dot_brk = interp1(lambda_traj,lambda_dot_traj,lambda_brk);
qIC = zeros(7,1);
qdotIC = qIC;
qddotIC = qIC;
qjerkIC = qIC;
tIC = interp1(lambda_traj,t_traj,lambda_brk);

for k = 1:7
    qtemp = interp1(lambda_traj,q(k,:),lambda_brk);
    qIC(k) = qtemp;
    qdot_temp = interp1(lambda_traj,qdot(k,:),lambda_brk);
    qdotIC(k) = qdot_temp;
    qddot_temp = interp1(lambda_traj,qddot(k,:),lambda_brk);
    qddotIC(k) = qddot_temp;
    qjerk_temp = interp1(lambda_traj,qjerk(k,:),lambda_brk);
    qjerkIC(k) = qjerk_temp;
end

% Find Motor Initial Conditions
tic

%load symbolic_results_simp.mat
load symbolic_results_bax.mat % alt2 is correct
Mddot_sym = Mddot; Mddot_diffq_sym = Mddot_diffq; 
Mddot_diffqdot_sym = Mddot_diffqdot; Mddot_diffqddot_sym = Mddot_diffqddot;
Mdot_sym = Mdot; MD_sym = MD; MDdot_sym = MDdot; 
MDdot_diffq_sym = MDdot_diffq; MDdot_diffqdot_sym = MDdot_diffqdot;
MDdot_diffqddot_sym = MDdot_diffqddot;
clear Mddot Mddot_diffq Mddot_diffqdot Mddot_diffqddot Mdot MD MDdot 
clear MDdot_diffq MDdot_diffqdot MDdot_diffqddot

load grasped_mass_symb_bax.mat
Adot_sym = Adot; Addot_sym = Addot; Addot_diffq_sym = Addot_diffq;
Addot_diffqdot_sym = Addot_diffqdot; 
Addot_diffqddot_sym = Addot_diffqddot; Ainvdot_sym = Ainvdot;
Ainvddot_sym = Ainvddot; Jdot_sym = Jdot; Jddot_sym = Jddot;
Jddot_diffq_sym = Jddot_diffq; Jddot_diffqdot_sym = Jddot_diffqdot;
Jddot_diffqddot_sym = Jddot_diffqddot;
clear Adot Addot Addot_diffq Addot_diffqdot Addot_diffqddot
clear Ainvdot Ainvddot Jdot Jddot Jddot_diffq Jddot_diffqdot Jddot_diffqddot

Ks = diag(robot_const.mprops.K);
Ns = diag(robot_const.mprops.N);
KN_inv = inv(Ks*Ns);

[MassInertia,Cq,F] = MCFgen_baxter(qIC,qdotIC,robot_const,zeros(6,1));
J = robotjacobian(robot_const.kin,qIC);

[Mdot, ~, ~, ~, ~,~] = ...
    mass_mat_derivs(Mdot_sym, Mddot_sym, Mddot_diffq_sym, ...
    Mddot_diffqdot_sym, Mddot_diffqddot_sym,qIC,qdotIC,qddotIC);

[f,~,fdot] = fderivs_func_fdot(qIC,qdotIC,qddotIC,qjerkIC,J,Mc,ric_tip,...
    robot_const,Jdot_sym,Jddot_sym,Adot_sym,Ainvdot_sym,Addot_sym,...
    Ainvddot_sym,Jddot_diffq_sym,Jddot_diffqdot_sym,Jddot_diffqddot_sym,...
    Addot_diffq_sym,Addot_diffqdot_sym,Addot_diffqddot_sym);

[Fdot, ~] = fric_derivs_bax(Ns*qddotIC,Ns*qjerk);

[C,Cdot,~] = C_mat_derivs(MD_sym,MDdot_sym,MDdot_diffq_sym,...
        MDdot_diffqdot_sym,MDdot_diffqddot_sym,qIC,qdotIC,qddotIC,qjerkIC);

Jdot = baxter_Jdiff(qIC,qdotIC);

qmIC = KN_inv*(MassInertia*qddotIC + Cq + F + Ks*Ns*Ns*qIC +J'*f);


qmdotIC = KN_inv*(Mdot*qddotIC + MassInertia*qjerkIC + Cdot*qdotIC + ...
    C*qddotIC + Fdot + Ks*Ns*Ns*qdotIC + Jdot'*f + J'*fdot);
toc
%% Run Simulation
sim('Baxter_brake_flex_sim.slx')
toc

%% Compute Task Space Trajectories After Braking

P0T_brk = zeros(3,length(t));
rpy_brk = zeros(3,length(t));
P0s_brk = zeros(3,length(t));
rpy_s_brk = zeros(3,length(t));

for k = 1:length(t)
    [Rtemp, ptemp] = fwdkin(robot_const.kin, q_brk(k,:));
    P0T_brk(:,k) = ptemp;
    P0s_brk(:,k) = ptemp + Rtemp*PTs;
    rpy_temp = rotm2eul(Rtemp,'XYZ');
    rpy_brk(:,k) = [rpy_temp(1);rpy_temp(2);rpy_temp(3)];
    Rstemp = Rtemp*RTs;
    rpy_s_temp = rotm2eul(Rstemp,'XYZ');
    rpy_s_brk(:,k) = [rpy_s_temp(1);rpy_s_temp(2);rpy_s_temp(3)];
    if k == 1
        R0T_initial = Rtemp;
        R0s_initial = Rstemp;
    elseif k == length(t)
        R0T_final = Rtemp;
        R0s_final = Rstemp;
    end
end


%% Plot Trajectories, Allow User to Enter lambda Choice for Initial Condition

% path variable space plot:
figure(1)
plot(lambda_traj,lambda_dot_traj,'b--','LineWidth',2)
hold on
scatter(lambda_brk,lambda_dot_brk,60,'r','filled')
xlabel('$\lambda$','Interpreter','Latex')
ylabel('$\dot{\lambda}$','Interpreter','Latex')

figure(2)
for k = 1:7
    if k==7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(t_traj,q(k,:),'b','LineWidth',2)
    hold on
    plot(t+tIC,q_brk(:,k),'r--','LineWidth',2)
    scatter(tIC,q_brk(1,k),30,'r','filled')
    xlabel('Time (s)','Interpreter','Latex')
    if (k==1)||(k==4)||(k==7)
        ylabel('Position (rad)')
    end
    title(['q',num2str(k)])
    if k==7
       legend('Desired Trajectory','Brake Trajectory') 
    end
end

figure(3)
task_space_titles = ["x","y","z","Roll","Pitch","Yaw"];
for k = 1:3
    subplot(2,3,k)
    plot(t_traj,P0T(k,:),'b','LineWidth',2)
    hold on
    plot(t+tIC,P0T_brk(k,:),'r--','LineWidth',2)
    scatter(tIC,P0T_brk(k,1),30,'r','filled')
    xlabel('Time (s)','Interpreter','Latex')
    if k == 1
        ylabel('Position (m)')
    end
    title(task_space_titles(k))
end
for k = 1:3
    subplot(2,3,k+3)
    plot(t_traj,rpy(k,:),'b','LineWidth',2)
    hold on
    plot(t+tIC,rpy_brk(k,:),'r--','LineWidth',2)
    scatter(tIC,rpy_brk(k,1),30,'r','filled')
    xlabel('Time (s)','Interpreter','Latex')
    if k == 1
        ylabel('Position (rad)')
    end
    title(task_space_titles(k+3))
    if k == 3
        legend('Desired Trajectory','Braking Trajectory')
    end
end

figure(4)
task_space_titles = ["x","y","z","Roll","Pitch","Yaw"];
for k = 1:3
    subplot(2,3,k)
    plot(t_traj,P0s(k,:),'b','LineWidth',2)
    hold on
    plot(t+tIC,P0s_brk(k,:),'r--','LineWidth',2)
    scatter(tIC,P0s_brk(k,1),30,'r','filled')
    xlabel('Time (s)','Interpreter','Latex')
    if k == 1
        ylabel('Object Position (m)')
    end
    title(task_space_titles(k))
end
for k = 1:3
    subplot(2,3,k+3)
    plot(t_traj,rpy_s(k,:),'b','LineWidth',2)
    hold on
    plot(t+tIC,rpy_s_brk(k,:),'r--','LineWidth',2)
    scatter(tIC,rpy_s_brk(k,1),30,'r','filled')
    xlabel('Time (s)','Interpreter','Latex')
    if k == 1
        ylabel('Object Position (rad)')
    end
    title(task_space_titles(k+3))
    if k == 3
        legend('Desired Trajectory','Braking Trajectory')
    end
end

%% Compute Deviation from Path
disp('Braking Position Error (end effector):')
e_brk = P0T_brk(:,end) - P0T_brk(:,1)
disp('Braking Position Error (object COM):')
e_obj_brk = P0s_brk(:,end) - P0s_brk(:,1)
disp('Braking Orientation Error (rpy), (end effector):')
ER_brk = R0T_initial*R0T_final';
e_rpy =  rotm2eul(ER_brk,'XYZ')'
disp('Braking Orientation Error (rpy), (object COM):')
ER_obj_brk = R0s_initial*R0s_final';
e_obj_rpu = rotm2eul(ER_obj_brk,'XYZ')'