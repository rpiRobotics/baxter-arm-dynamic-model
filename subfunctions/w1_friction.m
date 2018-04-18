function Tfric = w1_friction(qdot)

% Computes friction at the w1 joint of the Baxter arm using viscous friciton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coefficients:
b = -3;

Tfric = b*qdot;

end