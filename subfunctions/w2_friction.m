function Tfric = w2_friction(qdot)

% Computes friction at the w2 joint of the Baxter arm using viscous friciton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coefficients:
b = -3;

Tfric = b*qdot;

end