function f = end_effector_force_baxter(q,qdot,qddot,R0T,ric_const,robot_const,Mc,Ic)

% Generate spatial force applied by arm tip at the contact point for the
% Baxter arm

% inputs: 
% q: joint positions
% qdot: joint velocities
% qddot: joint accelerations
% R0T: rotation matrix from base frame to tip frame
% ric_const: vector from contact point to grasped object center of mass, in
%   the tip frame (constant vector - note that a rigid grasp with no 
%   changing grasp geometry is assumed)
% robot_const: robot definition structure, including mass properties
% Mc: mass-inertia matrix of the grasped object, represented in the object
%   center-of-mass frame
% Ic: inertia tensor of the grasped object, represented in the object
%   center-of-mass frame

% outputs:
% f: spatial force applied by manipulator arm on the grasped object at the
%   contact point (represented in the base frame)

A = [[R0T zeros(3)];[R0T*hat(ric_const) R0T]];
Adiff = baxter_Adiff(q,qdot,ric_const);
Jdiff = baxter_Jdiff(q,qdot);

J = robotjacobian(robot_const.kin,q);

Vc = A\J*qdot;
wc = [Vc(1);Vc(2);Vc(3)];

bc = [crossmat(wc)*Ic*wc;zeros(3,1)];
alphac = A\(Jdiff*qdot + J*qddot - Adiff*Vc);

f = (A')\(Mc*alphac + bc);

end
