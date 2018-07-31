function [MassInertia,Cq,F] = MCFgen_baxter(q,qdot,robot_const,V0)

% Generates joint-sapce mass-inertia matrix, coriolis/centrifugal terms,
% and friction terms for the Baxter rigid-joint dynamic model. The
% Newton-Euler algorithm is used to generate these dyanmic terms.

% inputs:
% q: joint positions
% qdot: joint velocities
% robot_const: robot structure (including dynamic parameters)
% V0: base velocity 
% 
% outputs:
% MassInertia: joint-space inertia matrix
% Cq: centrifugal/coriolis terms (note: Cq = C(q,qdot)*qdot)
% F: friction terms

% Dynamic Model:
% MassInertia*qddot + Cq + F = tau
% where qddot is joint accelerations and tau is applied motor torques

% Extract info from robot parameters structure
h = robot_const.kin.H; 
p = robot_const.kin.P;
I = robot_const.mprops.I;
m = robot_const.mprops.m;
c = robot_const.mprops.c;
N = robot_const.mprops.N;

% Generate H, PHI block matrices


H1 = [h(:,1); 0;0;0]; H2 = [h(:,2); 0;0;0]; H3 = [h(:,3); 0;0;0]; 
H4 = [h(:,4); 0;0;0]; H5 = [h(:,5); 0;0;0]; H6 = [h(:,6); 0;0;0]; 
H7 = [h(:,7); 0;0;0];

H = [H1 zeros(6,6); zeros(6,1) H2 zeros(6,5); zeros(6,2) H3 zeros(6,4); ...
    zeros(6,3) H4 zeros(6,3); zeros(6,4) H5 zeros(6,2); zeros(6,5) H6 ...
    zeros(6,1); zeros(6,6) H7];

PHI = phi_gen(q,h,p);

V = zeros(6,7);
Vtemp = V0;
Vprev = V0;
a = zeros(6,7);
b = zeros(6,7);
M = zeros(6,6,7);

for k = 1:7
    
   PHI_TEMP = PHI((6*(k) - 5):(6*(k)),(6*(k) - 5):(6*(k)));
   Htemp = H((6*(k) - 5):(6*(k)),k);
   Vtemp = PHI_TEMP*Vtemp + Htemp*qdot(k);
   V(:,k) = Vtemp;
   
   R = rot(H(:,k),q(k));
   
   a(:,k) = [crossmat(R'*Vprev(1:3))*(Vtemp(1:3));...
        R'*crossmat(Vprev(1:3))*(crossmat(Vprev(1:3))*p(:,k))];
   Vprev = Vtemp;
   
   b(:,k) = [crossmat(Vtemp(1:3))*I(:,:,k)*Vtemp(1:3); ...
        m(k)*crossmat(Vtemp(1:3))*crossmat(Vtemp(1:3))*c(:,k)];
    
    M(:,:,k) =  [I(:,:,k) m(k)*crossmat(c(:,k)); ...
        -m(k)*crossmat(c(:,k)) m(k)*eye(3,3)];
end

PHI = PHI(:,7:end);

% Solve for Mass Inertia matrix

M_NE = zeros(42,42);
M_NE(1:6,1:6) = M(:,:,1); M_NE(7:12,7:12) = M(:,:,2); 
M_NE(13:18,13:18) = M(:,:,3); M_NE(19:24,19:24) = M(:,:,4);
M_NE(25:30,25:30) = M(:,:,5); M_NE(31:36,31:36) = M(:,:,6); 
M_NE(37:42,37:42) = M(:,:,7);

MassInertia = H'*PHI'*M_NE*PHI*H;

% Solve for Coriolis/Centrifugal terms (C*qdot)
Cq = H'*PHI'*(M_NE*PHI*a(:) + b(:));

% Solve for Friction Terms
F = -friction_adjust_Baxter(zeros(7,1),N.*qdot);

end
