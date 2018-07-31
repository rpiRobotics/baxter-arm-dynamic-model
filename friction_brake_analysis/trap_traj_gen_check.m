% Trapezoidal Trajectory Generation

% This code generates a trapezoidal trajectory and checks the feasibility
% against the joint speed and torque constraints. This code includes the
% dynamics of a grasped load.

clear variables; close all; clc

% Set trapezoidal trajectory limits and sample time:
alimit = 0.15;
dlimit = 0.15;
vlimit = 0.3;
dt = 0.01;

% Define robot model:
[robot_const, ~] = defineBaxterSingleArmRigid();

% load path under consideration and generate delq^2/del lambda:
load ab_path_long.mat
delq = q_prime;
del2q = zeros(7,length(lambda));
N = length(lambda)-1;
for k = 1:7
    delqtemp = delq(k,:);
    del2qtemp = gradient(delqtemp,(1/N));
    del2q(k,:) = del2qtemp;
end

% Define grasped object geometry and mass properties:
% (Currently set for air bearing object)
m = 14.22; % mass (kg)
r = 0.4318/2; % radius
ht = 0.2794; % height
Ic = [(0.5*m*(3*r^2 + ht^2)) 0 0; 0 (0.5*m*(3*r^2 + ht^2)) 0; ...
    0 0 0.5*m*r^2]; % cylinder model
RTs = rotz(-42*pi/180); % end effector to object center rotation matrix
PTs = RTs*[-0.28;0.35079;-0.06]; % end effector to object center vector
ric_tip = PTs;
Mc = [[Ic zeros(3)];[zeros(3) m*eye(3)] ]; % mass-inertia
Mc_inv = inv(Mc); % inverse of mass-inertia matrix

% Generate trapezoidal trajectory:
% inputs: (xo,xf,vo,vf,vmax,amax,dmax,t)
% xo: initial position
% xf: final position
% vo: initial velocity
% vf: final velocity
% vmax: maximum velocity
% amax: max acceleration
% dmax: max decel (magnitude)
% t: sample time 

[~,~,~,ta,tb,tf]=trapgen(0,1,0,0,vlimit,alimit,dlimit,0);

t_traj = [0:dt:tf];
lambda_traj = zeros(1,length(t_traj));
lambda_dot_traj = lambda_traj;
lambda_ddot_traj = lambda_traj;

for k = 1:length(t_traj)
    [xtemp,vtemp,atemp,~,~,~]=trapgen(0,1,0,0,vlimit,alimit,dlimit,t_traj(k));
    lambda_traj(k) = xtemp;
    lambda_dot_traj(k) = vtemp;
    lambda_ddot_traj(k) = atemp;
end

% Generate Joint Space Trajectory:
q = zeros(7,length(lambda_traj));
qdot = q;
qddot = q;
for k = 1:length(t_traj)
    for g = 1:7
        qtemp = interp1(lambda,q_lambda(g,:),lambda_traj(k),'linear',...
            q_lambda(g,end));
        q(g,k) = qtemp;
        delqtemp = interp1(lambda,delq(g,:),lambda_traj(k),'linear',...
            delq(g,end));
        del2qtemp = interp1(lambda,del2q(g,:),lambda_traj(k),'linear',...
            del2q(g,end));
        qdot(g,k) = delqtemp*lambda_dot_traj(k);
        qddot(g,k) = del2qtemp*lambda_dot_traj(k)^2 + ...
            delqtemp*lambda_ddot_traj(k);
    end
    
end

% Compute joint actuation
tau = zeros(7,length(lambda_traj));
for k = 1:(length(lambda_traj))
    [Rtemp, ptemp] = fwdkin(robot_const.kin, q(:,k));
    J = robotjacobian(robot_const.kin, q(:,k));
    [M,Cq,D] = MCFgen_baxter(q(:,k),qdot(:,k),robot_const,zeros(6,1));
     f = end_effector_force_baxter(q(:,k),qdot(:,k),qddot(:,k),Rtemp,...
         ric_tip,robot_const,Mc,Ic);
     
    tau(:,k) = M*qddot(:,k) + Cq + D + J'*f;
end

% Plot trajectory (in lambda space) for check:
figure(1)
subplot(2,2,1)
plot(t_traj,lambda_ddot_traj,'b','LineWidth',2)
hold on
plot([0,t_traj(end)],[alimit,alimit],'k--','LineWidth',2)
plot([0,t_traj(end)],[-dlimit,-dlimit],'k--','LineWidth',2)
xlabel('t (s)','Interpreter','Latex')
ylabel('$\ddot{\lambda}$','Interpreter','Latex')
subplot(2,2,2)
plot(t_traj,lambda_dot_traj,'b','LineWidth',2)
hold on
plot([0,t_traj(end)],[vlimit,vlimit],'k--','LineWidth',2)
xlabel('t (s)','Interpreter','Latex')
ylabel('$\dot{\lambda}$','Interpreter','Latex')
subplot(2,2,3)
plot(t_traj,lambda_traj,'b','LineWidth',2)
hold on
plot([0,t_traj(end)],[1,1],'k--','LineWidth',2)
xlabel('t (s)','Interpreter','Latex')
ylabel('$\lambda$','Interpreter','Latex')
legend('Trajectory','Limit')

% Plot joint space trajectory to ensure limits are obeyed
figure(2)
for k = 1:7
    if k==7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(lambda_traj,q(k,:),'b','LineWidth',2)
    hold on
    plot([0,1],[robot_const.limit.lower_joint_limit(k),...
        robot_const.limit.lower_joint_limit(k)],'k--','LineWidth',2)
    plot([0,1],[robot_const.limit.upper_joint_limit(k),...
        robot_const.limit.upper_joint_limit(k)],'k--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    if (k==1)||(k==4)||(k==7)
        ylabel('Position (rad)')
    end
    title(['q',num2str(k)])
    if k==7
        legend('Joint Position','Limits')
    end
end

figure(3)
for k = 1:7
    if k==7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(lambda_traj,qdot(k,:),'b','LineWidth',2)
    hold on
    plot([0,1],[-robot_const.limit.velocity_limit(k),...
        -robot_const.limit.velocity_limit(k)],'k--','LineWidth',2)
    plot([0,1],[robot_const.limit.velocity_limit(k),...
        robot_const.limit.velocity_limit(k)],'k--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    if (k==1)||(k==4)||(k==7)
        ylabel('Velocity (rad/s)')
    end
    title(['q',num2str(k)])
    if k==7
        legend('Joint Velocity','Limits')
    end
end

figure(4)
for k = 1:7
    if k==7
        subplot(3,3,8)
    else
        subplot(3,3,k)
    end
    plot(lambda_traj,tau(k,:),'b','LineWidth',2)
    hold on
    plot([0,1],[-robot_const.limit.effort_limit(k),...
        -robot_const.limit.effort_limit(k)],'k--','LineWidth',2)
    plot([0,1],[robot_const.limit.effort_limit(k),...
        robot_const.limit.effort_limit(k)],'k--','LineWidth',2)
    xlabel('$\lambda$','Interpreter','Latex')
    if (k==1)||(k==4)||(k==7)
        ylabel('Torque (Nm)')
    end
    title(['q',num2str(k)])
    if k==7
        legend('Joint Effort','Limits')
    end
end