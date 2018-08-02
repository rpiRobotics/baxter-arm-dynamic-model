function [Fdot, Fddot] = fric_derivs_bax(qddot,qdot3)
% This function estimates derivatives of the joint friction vector

F = diag([7 7 5 5 3 3 3]);

Fdot = F*qddot;
Fddot = F*qdot3;

end