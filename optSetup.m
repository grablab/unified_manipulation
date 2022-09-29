clear
clc
close all

% addpath("tests")
% % % % restoredefaultpath

%% Running for ALL hand varieties

% % LOAD [goal_xyz, goal_ang, env, taskname_add] first

hand_arr = {'fixed','spherical','modelO','modelQ','stewart'};
% hand_arr = {'fixed'}; % debugging

for hh = 1:length(hand_arr)

    hand = hand_arr{hh};
    disp(strcat('Running for hand: ', hand));
    
    % % Load Robot
    robotname = 'universalUR10'; %'kukaIiwa14';
    
    [hand_type, hand_ranges, handDOFs] = loadHand(hand);
    
    [robot, arm_eef_name, hand_name] = loadRobotBody(robotname, hand_type, hand_ranges);
    
    robHomeCfg = robot.homeConfiguration;
    numJointsArm = length(robHomeCfg) - handDOFs;
    armHomeXYZ = tform2trvec(getTransform(robot,robHomeCfg,arm_eef_name));
    [~,sepAtHome] = checkCollision(robot,robHomeCfg, env, 'Exhaustive','on');
    sepAtHome = filterSepDist(sepAtHome);

    invKinRob = inverseKinematics('RigidBodyTree', robot);
    tformStart = eul2tform(goal_ang(1,:), 'XYZ');
    tformStart(1:3, end) = goal_xyz(1,:)';
    [robSeedCfg,~] = invKinRob(hand_name, tformStart, ones(1,6), robHomeCfg);
    
    switch robotname
        case 'kukaIiwa14'
            armjointLim_upper = [170 120 170 120 170 120 175].*pi/180;
            armjointLim_lower = -armjointLim_upper;
        case 'kinovaMicoM1N4S200'
            armjointLim_upper = [180 310 335 180].*pi/180;
            armjointLim_lower = -[180 50 25 180].*pi/180;
        case 'wam4'
            armjointLim_upper = [150 113 157 180].*pi/180;
            armjointLim_lower = -[150 113 157 50].*pi/180;
        case 'universalUR10'
            armjointLim_upper =  ones(1, numJointsArm)*2*pi;
            armjointLim_lower = -ones(1, numJointsArm)*2*pi;
        otherwise
            armjointLim_upper = ones(1, numJointsArm)*pi;
            armjointLim_lower = -armjointLim_upper;
    end
    
    close all
    
    % % Variables Setup
    global robstruct
    robstruct.robot = robot;
    
    robstruct.goal_xyz = [0 0 0];
    robstruct.goal_R = eul2rotm([0     0     0], 'XYZ');
    robstruct.prevCfg = repmat(robSeedCfg, 4, 1);
    
%     robstruct.max_vel = deg2rad([85 85 100 75 130 135 135]);    % for kukaIiwa14
%     robstruct.max_torque = [320 320 176 176 110 40 40]; %Nm     % for kukaIiwa14

    robstruct.max_vel = deg2rad([120 120 180 180 180 180]);     % for universalUR10
    robstruct.max_torque = [330 330 150 56 56 56]; %Nm          % for universalUR10

    robstruct.max_acc = deg2rad(500*ones(1, numJointsArm));
    robstruct.max_jerk = deg2rad(800*ones(1, numJointsArm));
    
    robstruct.max_obj_jerk = 5;     % m/s^3
    robstruct.h = 3;
    robstruct.armJntLim_lo = armjointLim_lower;
    robstruct.armJntLim_up = armjointLim_upper;
    robstruct.arm_eef_name = arm_eef_name;
    robstruct.handDOFs = handDOFs;
    robstruct.hand = hand;
    robstruct.hand_name = hand_name;
    robstruct.hand_ranges = hand_ranges(hand_type>0);
    robstruct.sepAtHome = sepAtHome;
    robstruct.env = env;
    
    % % Problem Setup
    
    problem.objective = 'objFun';
    problem.x0 = robstruct.prevCfg(1, :);
    problem.lb = [robstruct.armJntLim_lo, -robstruct.hand_ranges];
    problem.ub = [robstruct.armJntLim_up, robstruct.hand_ranges];
    problem.solver = 'fmincon';
    problem.options = optimoptions('fmincon', 'Algorithm', 'interior-point', ...
        'StepTolerance',1e-11, 'MaxFunctionEvaluations', 1e10, ...
         'MaxIterations', 1e4, 'Display','none');
    
    
    % % Run
    
    objPoses_arr.xyz = goal_xyz;
    objPoses_arr.ang = goal_ang;
    
    nPoints = size(objPoses_arr.xyz,1);
    prevCfg_all = robstruct.prevCfg;
    
    exitflag = NaN*zeros(nPoints,1);
    fval = NaN*zeros(nPoints,1);

    invKinRob = inverseKinematics('RigidBodyTree', robot, 'SolverAlgorithm', 'LevenbergMarquardt');
    
    disp('Percent Complete...')
    
    for oo = 1:nPoints
        robstruct.goal_xyz = objPoses_arr.xyz(oo, :);
        robstruct.goal_R = eul2rotm(objPoses_arr.ang(oo, :), 'XYZ');

        tformx0 = rotm2tform(robstruct.goal_R);
        tformx0(1:3, end) = robstruct.goal_xyz';
        [robx0Cfg,~] = invKinRob(robstruct.hand_name, tformx0, ones(1,6), robstruct.prevCfg(1, :));
        problem.x0 = robx0Cfg;
    
        [x, fval(oo), exitflag(oo), output] = fmincon(problem);
    
        prevCfg_all = [x; prevCfg_all];
    
        robstruct.prevCfg = prevCfg_all(1:4,:);
    
        disp(100.*oo/nPoints);
    
    end
    
    robstruct.exitflag = exitflag;
    robstruct.fval = fval;
    
    disp('Completed.')
    
    
    filename = strcat('tests/', taskname_add, '/test_', robotname, '_', robstruct.hand, '.mat');

    save(filename, 'objPoses_arr', 'prevCfg_all', 'problem', 'robstruct');

end

