function Tfric = s0_friction(qdot)
% Computes friction at the s0 joint of the Baxter arm using viscous friciton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

b = -7;

Tfric = b*qdot;


end