function Tfric = e1_friction(qdot)

% Computes friction at the s0 joint of the Baxter arm using viscous
% friction model

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coefficients:
b = -5;

Tfric = b*qdot;

end