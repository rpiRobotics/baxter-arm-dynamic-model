function [robot_const, robot_structure] = defineBaxterSingleArmFlex(varargin)

% This function generates constant parameters associated with a single 
% Baxter manipulator arm. Kinematics and mass properties are contained 
% within the constant structure. Parameters based on Rethink Robotics 
% URDF files.

    %
    % [robot_const, robot_structure] = defineBaxterSingleArm()
    % [robot_const, robot_structure] = defineBaxterSingleArm(...)
    %                           allows additional optional parameters
    %       'Origin'        :   default [eye(3) [0;0;0]; [0 0 0] 1]
    %
    % define-file for a single Baxter arm. Returns struct array with 
    %   the following fields:
    % root
    %   -> name             : (1) 'robot_arm'
    %   -> kin
    %       -> H            : joint axes
    %       -> P            : inter-joint translation
    %       -> joint_type   : joint types
    %   -> mprops
    %       -> m            : link masses
    %       -> c            : COM vector
    %       -> I            : link inertia matrices
    %       -> N            : Gear Ratios
    %       -> K            : Spring Constants for flexible joints
    %   -> limit
    %       -> upper_joint_limit :  upper joint limits [rad]
    %       -> lower_joint_limit :  lower joint limits [rad]
    %       -> velocity_limit:   :  velocity limits [rad/s]
    %       %% CURRENTLY NO LIMITS SET %%
    %
    %   -> vis
    %       -> joints       :  cell array each joint [m]
    %       -> links        :  height of each joint [m]
    %       -> frame        :  frame dimensions
    %       -> peripherals  :  empty
    %
    
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    x0 = [1;0;0]; y0 = [0;1;0]; z0 = [0;0;1]; zed = [0;0;0];

    flags = {'Origin'};
    defaults = {[eye(3) zed; zed' 1]};
    opt_values = mrbv_parse_input(varargin, flags, defaults);
    origin = opt_values{1};
    R0 = origin(1:3,1:3); t0 = origin(1:3,4);
    
    % set limits
    upper_joint_limit = [97.5; 60; 175; 150; 175; 120; 175]*pi/180;
    lower_joint_limit = [-97.5;-123; -175; -2.5; -175; -90; -175]*pi/180;
    velocity_limit = [2.5;2.5;2.5;2.5;5;5;5];
    torque_limit = [60;60;60;60;20;20;20];
    
    
    % set visualization parameters
    joint_radius = [.075;.075;.075;.07;.07;.06;.06];
    joint_height = [.1;.19;.15;.18;.14;.15;.12];
    link_type1_props = {'FaceColor', [.9;0;0], ...
                            'EdgeAlpha', 0};
    link_type2_props = {'FaceColor', [.2;0.2;0.2], ...
                            'EdgeAlpha', 0};
    joint_type1_props = {'FaceColor', [0.2;0.2;0.2]};
    joint_type2_props = {'FaceColor', [0.9;0;0]};
    
    arm_mount_props = {'FaceColor',[0.2;0.2;0.2],'EdgeAlpha',0};
    arm_mount_param = struct('width',0.2,'length',0.15,'height',0.05);
    % Grab standard robot structure
    robot_const = defineEmptyRobot(1);
    
    robot_const.name = 'Single Baxter Arm';
    
    % kinematics
    robot_const(1).kin.H = R0*[z0 y0 x0 y0 x0 y0 x0];
    robot_const(1).kin.P = R0*[[0.06375;.25888;0.119217], ...
                            [[0.069;0;0.27035], ...
                            zed, [0.36435;0;-0.069], ...
                            zed, [0.37429;0;-0.01], ...
                            zed, 0.229525*x0]];
    robot_const(1).kin.P(:,1) = t0 + robot_const(1).kin.P(:,1);
    robot_const(1).kin.joint_type = zeros(1,7);
    
    % Dynamic Limits
    robot_const(1).limit.upper_joint_limit = upper_joint_limit;
    robot_const(1).limit.lower_joint_limit = lower_joint_limit;
    robot_const(1).limit.velocity_limit = velocity_limit;
    robot_const(1).limit.effort_limit = torque_limit;
    
    % mass and inertia properties
    % Link masses:
    m = [5.70044, 3.22698, 4.31272, 2.07206, 2.24665, 1.60979, 0.35093];
    robot_const(1).mprops.m = m;
    
    % Link COM:
    % COM in Rethink Robotics frames:
    cx1 = 0.01783; cy1 = 0.00086; cz1 = 0.19127; cx2 = 0.06845; 
    cy2 = 0.00269; cz2 = -0.00529; cx3 = -0.00276; cy3 = 0.00132; cz3 = 0.18086;
    cx4 = -0.06; cy4 = 0; cz4 = 0; cx5 = -0.00168; 
    cy5 = 0.0046; cz5 = 0.13952; cx6 = 0.06041; cy6 = 0.00697; 
    cz6 = 0.006; cx7 = 0.00198; cy7 = 0.00125; cz7 = 0.01855;
    c1_prime = [cx1;cy1;cz1]; c2_prime = [cx2;cy2;cz2]; 
    c3_prime = [cx3;cy3;cz3]; c4_prime = [cx4;cy4;cz4];
    c5_prime = [cx5;cy5;cz5]; c6_prime = [cx6;cy6;cz6];
    c7_prime = [cx7;cy7;cz7];
    
    % COM in PoE frames (match kinematics in this file):
    c1 = c1_prime; c2 = [1 0 0; 0 0 1; 0 -1 0]*c2_prime; 
    c3 = [0.102;0;0] + [0 0 1; 0 1 0; -1 0 0]*c3_prime; 
    c4 = [1 0 0; 0 0 1; 0 -1 0]*c4_prime;
    c5 = [0.10359;0;0] + [0 0 1; 0 1 0; -1 0 0]*c5_prime; 
    c6 = [1 0 0; 0 0 1; 0 -1 0]*c6_prime;
    c7 = [0.115975;0;0] + [0 0 1; 0 1 0; -1 0 0]*c7_prime;

    c = [c1, c2, c3, c4, c5, c6, c7];
    robot_const(1).mprops.c = c;
    
    % Inertia Matrices in Rethink Robotics frames:
    Ic1_1_prime = [0.04709102262, 0.00012787556, 0.00614870039;...
        0.00012787556, 0.03766976455, 0.00078086899;...
        0.00614870039, 0.00078086899, 0.03595988478];
    Ic2_2_prime = [0.01175209419, -0.00030096398, 0.00207675762;...
        -0.00030096398, 0.0278859752, -0.00018821993; ...
        0.00207675762, -0.00018821993, 0.02078749298];
    Ic3_3_prime = [0.02661733557, 0.00029270634, 0.00392189887;...
        0.00029270634, 0.02844355207, 0.0010838933;...
        0.00392189887, 0.0010838933, 0.01248008322];
    Ic4_4_prime = [0.00711582686, 0.00036036173, 0.0007459496;...
        0.00036036173, 0.01318227876, -0.00019663418; ...
        0.0007459496, -0.00019663418, 0.00926852064];
    Ic5_5_prime = [0.01667742825, 0.00018403705, 0.00018657629;...
        0.00018403705, 0.01675457264, -0.00064732352; ...
        0.00018657629, -0.00064732352, 0.0037463115];
    Ic6_6_prime = [0.00387607152, -0.00044384784, -0.00021115038;...
        -0.00044384784, 0.00700537914, 0.00015348067;...
        -0.00021115038, 0.00015348067, 0.0055275524];
    Ic7_7_prime = [0.00025289155, 0.00000575311, -0.00000159345;...
        0.00000575311, 0.0002688601, -0.00000519818;...
        -0.00000159345, -0.00000519818, 0.0003074118];
    

    % Motor inertias (assumed values)
    Im1 = 0.5*10^(-3.5);
    Im2 = 0.5*10^(-3.5);
    Im3 = 0.5*10^(-3.5);
    Im4 = 0.5*10^(-3.5);
    Im5 = 0.5*10^(-3.5);
    Im6 = 0.5*10^(-3.5);
    Im7 = 0.5*10^(-3.5);

    % Specify Gear Ratios (can change in the future)
    N1 = 1; N2 = 1; N3 = 1; N4 = 1; N5 = 1; N6 = 1; N7 = 1;
    
    % Inertia matrices in PoE frames (to match this files kinematics):
    Ic1_1 = Ic1_1_prime;
    Ic2_2 = [1 0 0; 0 0 1; 0 -1 0]*Ic2_2_prime*[1 0 0; 0 0 1; 0 -1 0]';
    Ic3_3 = [0 0 1; 0 1 0; -1 0 0]*Ic3_3_prime*[0 0 1; 0 1 0; -1 0 0]';
    Ic4_4 = [1 0 0; 0 0 1; 0 -1 0]*Ic4_4_prime*[1 0 0; 0 0 1; 0 -1 0]';
    Ic5_5 = [0 0 1; 0 1 0; -1 0 0]*Ic5_5_prime*[0 0 1; 0 1 0; -1 0 0]';
    Ic6_6 = [1 0 0; 0 0 1; 0 -1 0]*Ic6_6_prime*[1 0 0; 0 0 1; 0 -1 0]';
    Ic7_7 = [0 0 1; 0 1 0; -1 0 0]*Ic7_7_prime*[0 0 1; 0 1 0; -1 0 0]';
    Ic(:,:,1) = Ic1_1; Ic(:,:,2) = Ic2_2; Ic(:,:,3) = Ic3_3; 
    Ic(:,:,4) = Ic4_4; Ic(:,:,5) = Ic5_5; Ic(:,:,6) = Ic6_6; 
    Ic(:,:,7) = Ic7_7;
    for k = 1:7
        I(:,:,k) = Ic(:,:,k) - m(k)*(hat(c(:,k))*hat(c(:,k)));
    end
    
    
    % Motor inertia corrections (PoE frames)
    %%% SUBTRACT MOTOR INERTIAS DUE TO ASSUMPTION BASE VALUES ALREADY
    %%% CONTAIN THE CORRECTION - NOT CONFIRMED WITH RETHINK ROBOTICS
    I(:,:,1) = I(:,:,1) - Im1*N1^2*(z0*z0.');
    I(:,:,2) = I(:,:,2) - Im1*N2^2*(y0*y0.');
    I(:,:,3) = I(:,:,3) - Im2*N3^2*(x0*x0.');
    I(:,:,4) = I(:,:,4) - Im2*N4^2*(y0*y0.');
    I(:,:,5) = I(:,:,5) - Im3*N5^2*(x0*x0.');
    I(:,:,6) = I(:,:,6) - Im3*N6^2*(y0*y0.');
    I(:,:,7) = I(:,:,7) - Im3*N7^2*(x0*x0.');
    robot_const(1).mprops.I = I;
    
    robot_const(1).mprops.Im = [Im1 Im2 Im3 Im4 Im5 Im6 Im7];
    
    robot_const(1).mprops.N = [N1; N1; N2; N2; N3; N3; N3];
    
    robot_const(1).mprops.K = [843 843 843 843 250 250 250];
    
    % visualization
    robot_const(1).vis.joints = ...
        struct('param',cell(1,7),'props',cell(1,7));
    for n=1:7
        robot_const(1).vis.joints(n).param = ...
                                    struct('radius',joint_radius(n), ...
                                           'height',joint_height(n));
        robot_const(1).vis.joints(n).props = joint_type1_props;
    end
    
    robot_const(1).vis.joints(2).props = joint_type2_props;
    robot_const(1).vis.joints(3).props = joint_type2_props;
    robot_const(1).vis.joints(5).props = joint_type2_props;
    
    robot_const(1).vis.links = struct('handle', cell(1,8), ...
                                'R', cell(1,8), 't', cell(1,8), ...
                                'param',cell(1,8),'props',cell(1,8));
                                                   
    
    robot_const(1).vis.links(2).handle = @createCylinder;
    robot_const(1).vis.links(2).R = R0*eye(3);
    robot_const(1).vis.links(2).t = t0 + R0*[0;0;0.1777];
    robot_const(1).vis.links(2).param = struct('radius',0.075,'height',0.2553);
    robot_const(1).vis.links(2).props = link_type2_props;                        
    
    robot_const(1).vis.links(4).handle = @createCylinder;
    robot_const(1).vis.links(4).R = R0*rot(y0,pi/2);
    robot_const(1).vis.links(4).t = R0*[.1847;0;0];
    robot_const(1).vis.links(4).param = struct('radius',0.075,'height',.2193);
    robot_const(1).vis.links(4).props = link_type1_props;

    robot_const(1).vis.links(6).handle = @createCylinder;
    robot_const(1).vis.links(6).R = R0*rot(y0,pi/2);
    robot_const(1).vis.links(6).t = R0*[.1921;0;0];
    robot_const(1).vis.links(6).param = struct('radius',0.07,'height',.2443);
    robot_const(1).vis.links(6).props = link_type1_props;

    robot_const(1).vis.links(8).handle = @createCylinder;
    robot_const(1).vis.links(8).R = R0*rot(y0,pi/2);
    robot_const(1).vis.links(8).t = R0*[.1448;0;0];
    robot_const(1).vis.links(8).param = struct('radius',0.05,'height',.1695);
    robot_const(1).vis.links(8).props = link_type2_props;
    
    % Define dimensions of coordinate frame
    robot_const(1).vis.frame = struct('scale',0.2,'width',0.01);
    
    % Define structure for full combined robot
    robot_structure = defineEmptyRobotStructure(1);
    robot_structure.name = robot_const.name;
    [robot_structure.create_properties] = deal({'CreateFrames','on'});


end