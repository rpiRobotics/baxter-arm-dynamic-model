%  Generates symbolic Jacobian A matrix differentiation
% Turn into a function using f = matlabFunction(Adiff,'File','filename')

clear variables
close all
clc

syms q1 q2 q3 q4 q5 q6 q7 real
syms qdot1 qdot2 qdot3 qdot4 qdot5 qdot6 qdot7 real
syms r1 r2 r3 real % constant values of ric in tip frame 
r = [r1;r2;r3];
q = [q1 q2 q3 q4 q5 q6 q7];
qdot = [qdot1 qdot2 qdot3 qdot4 qdot5 qdot6 qdot7];
[robot_const, robot_structure] = defineBaxterSingleArmRigid();

h1 = robot_const.kin.H(:,1); h2 = robot_const.kin.H(:,2);
h3 = robot_const.kin.H(:,3); h4 = robot_const.kin.H(:,4);
h5 = robot_const.kin.H(:,5); h6 = robot_const.kin.H(:,6);
h7 = robot_const.kin.H(:,7);

p01 = robot_const.kin.P(:,1); p12 = robot_const.kin.P(:,2);
p23 = robot_const.kin.P(:,3); p34 = robot_const.kin.P(:,4);
p45 = robot_const.kin.P(:,5); p56 = robot_const.kin.P(:,6);
p67 = robot_const.kin.P(:,7); p7T = robot_const.kin.P(:,8);

R01 = rot(h1,q1); R12 = rot(h2,q2); R23 = rot(h3,q3); R34 = rot(h4,q4);
R45 = rot(h5,q5); R56 = rot(h6,q6); R67 = rot(h7,q7); R7T = eye(3);

R02 = R01*R12; R03 = R02*R23; R04 = R03*R34; R05 = R04*R45; R06 = R05*R56;
R07 = R06*R67; R0T = R07*R7T;

A = [eye(3) zeros(3); hat(-R0T*r) eye(3)];

Adiff = zeros(6,6);

for k = 1:7
   Adiff = Adiff + diff(A,q(k))*qdot(k); 
end