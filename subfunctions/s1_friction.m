function Tfric = s1_friction(qdot)

% Computes friction at the s1 joint of the Baxter arm using viscous friciton


% coefficients:
b = -7;

Tfric = b*qdot;

end