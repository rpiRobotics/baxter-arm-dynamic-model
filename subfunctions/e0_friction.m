function Tfric = e0_friction(qdot)

% Computes friction at the e0 joint of the Baxter arm using viscous friciton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coefficients:
b = -5;

Tfric = b*qdot;

end