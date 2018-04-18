% Generates symbolic Jacobian differentiation
% Turn into a function
clear variables
close all
clc

syms q1 q2 q3 q4 q5 q6 q7 real
syms qdot1 qdot2 qdot3 qdot4 qdot5 qdot6 qdot7 real
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

P1T = R01*p12 + R02*p23 + R03*p34 + R04*p45 + R05*p56 + R06*p67 + R07*p7T;
P2T = R02*p23 + R03*p34 + R04*p45 + R05*p56 + R06*p67 + R07*p7T;
P3T = R03*p34 + R04*p45 + R05*p56 + R06*p67 + R07*p7T;
P4T = R04*p45 + R05*p56 + R06*p67 + R07*p7T;
P5T = R05*p56 + R06*p67 + R07*p7T;
P6T = R06*p67 + R07*p7T;
P7T = R07*p7T;

J = [[h1 R01*h2 R02*h3 R03*h4 R04*h5 R05*h6 R06*h7];[hat(h1)*P1T ...
    hat(R01*h2)*P2T hat(R02*h3)*P3T hat(R03*h4)*P4T hat(R04*h5)*P5T ...
    hat(R05*h6)*P6T hat(R06*h7)*P7T]];

Jdiff = zeros(6,7);

for k = 1:7
   Jdiff = Jdiff + diff(J,q(k))*qdot(k); 
end
