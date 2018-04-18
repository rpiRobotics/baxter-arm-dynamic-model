function Tfric = w0_friction(qdot)

% Computes friction at the wp joint of the Baxter arm using viscous friciton

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% coefficients:
b = -3;

Tfric = b*qdot;

end