function tau = friction_adjust_Baxter(tau,qdot)

% Correct joint torques for frictional effects. This adjustment uses a
% viscous friction model.
% Can also be used to solve for friction terms based on joint velocity - to
% do so, send zero vector for tau input
% This function currently uses toy friction models - will have to be
% updated in the future
%
%
% Inputs:
% tau: joint torques (7x1), not adjusted for friction
% qdot: joint velocities (7x1)
%
% Outputs:
% tau: joint torques (7x1), adjusted for friction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Compute torque reduction due to friction:
    [Tfric7] = w2_friction(qdot(7));
    [Tfric6] = w1_friction(qdot(6));
    [Tfric5] = w0_friction(qdot(5));
    [Tfric4] = e1_friction(qdot(4));
    [Tfric3] = e0_friction(qdot(3));
    [Tfric2] = s1_friction(qdot(2));
    [Tfric1] = s0_friction(qdot(1));
    
    % Adjust torques at joints:
    tau(7) = tau(7) + Tfric7;
    tau(6) = tau(6) + Tfric6;
    tau(5) = tau(5) + Tfric5;
    tau(4) = tau(4) + Tfric4;
    tau(3) = tau(3) + Tfric3;
    tau(2) = tau(2) + Tfric2;
    tau(1) = tau(1) + Tfric1;

end