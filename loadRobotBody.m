function [robot, arm_eef_name, hand_name] = loadRobotBody(robotname, handType, handRanges)
    
    if nargin<2
        handType = zeros(1,6);       % 0 - no dof, 1:dof at XYZXrYrZr axis
        if nargin < 1
            robotname = 'kukaIiwa14';     % universalUR5 % kukaIiwa14 % frankaEmikaPanda
        end
    end

    switch robotname
        case 'universalUR5'
            robot = importrobot('universalUR5.urdf', 'DataFormat', 'row');
        case 'universalUR10'
            robot = importrobot('universalUR10.urdf', 'DataFormat', 'row');
        case 'kinovaMicoM1N4S200'
            robot = importrobot('kinovaMicoM1N4S200.urdf', 'DataFormat', 'row');
            removeBody(robot, 'm1n4s200_link_finger_1');        % removing gripper finger 1 joint
            removeBody(robot, 'm1n4s200_link_finger_2');        % removing gripper finger 2 joint
        case 'wam7'
            robot = importrobot('data/armConfigs data/WAM data/barrett_model/robots/wam7.urdf', 'DataFormat', 'row');
        case 'wam4'
            robot = importrobot('data/armConfigs data/WAM data/barrett_model/robots/wam7.urdf', 'DataFormat', 'row');
            robot0 = copy(robot);
            removeBody(robot0, 'wam/wrist_yaw_link');
            newBody = rigidBody('wam/eef');
            newBody.Joint = rigidBodyJoint('wam/eef_fix','fixed');
            setFixedTransform(newBody.Joint, robot.Bodies{6}.Joint.JointToParentTransform);
            addBody(robot0, newBody, robot.Bodies{5}.Name);
            robot = copy(robot0);

        otherwise
            robot = loadrobot(robotname,'DataFormat','row');
    end

    robot0 = copy(robot);


    %%% Returning robot_eef_name
    if strcmp(robotname, 'frankaEmikaPanda')
        arm_eef_name = robot.BodyNames{9};
    else
        arm_eef_name = robot.BodyNames{end};
    end
    hand_name = arm_eef_name;
    hand_exist_flag = 0;

    %%% Adding a hand based on handType
    if handType(1)==1
        newBody = rigidBody('handX');
        newBody.Joint = rigidBodyJoint('hand_jointX','prismatic');
        newBody.Joint.JointAxis = [1 0 0];
        newBody.Joint.PositionLimits = [-handRanges(1), handRanges(1)];
        if hand_exist_flag == 0
            newJointTransform = eye(4);
            newJointTransform(3,4) = 0.1;       % hand is 0.1m above arm eef
            setFixedTransform(newBody.Joint, newJointTransform);
            hand_exist_flag = 1;
        end
        addBody(robot0, newBody, hand_name);
        hand_name = newBody.Name;
    end
    if handType(2)==1
        newBody = rigidBody('handY');
        newBody.Joint = rigidBodyJoint('hand_jointY','prismatic');
        newBody.Joint.JointAxis = [0 1 0];
        newBody.Joint.PositionLimits = [-handRanges(2), handRanges(2)];
        if hand_exist_flag == 0
            newJointTransform = eye(4);
            newJointTransform(3,4) = 0.1;       % hand is 0.1m above arm eef
            setFixedTransform(newBody.Joint, newJointTransform);
            hand_exist_flag = 1;
        end
        addBody(robot0, newBody, hand_name);
        hand_name = newBody.Name;
    end
    if handType(3)==1
        newBody = rigidBody('handZ');
        newBody.Joint = rigidBodyJoint('hand_jointZ','prismatic');
        newBody.Joint.JointAxis = [0 0 1];
        newBody.Joint.PositionLimits = [-handRanges(3), handRanges(3)];
        if hand_exist_flag == 0
            newJointTransform = eye(4);
            newJointTransform(3,4) = 0.1;       % hand is 0.1m above arm eef
            setFixedTransform(newBody.Joint, newJointTransform);
            hand_exist_flag = 1;
        end
        addBody(robot0, newBody, hand_name);
        hand_name = newBody.Name;
    end
    
    if handType(4)==1
        newBody = rigidBody('handXr');
        newBody.Joint = rigidBodyJoint('hand_jointXR','revolute');
        newBody.Joint.JointAxis = [1 0 0];
        newBody.Joint.PositionLimits = [-handRanges(4), handRanges(4)];
        if hand_exist_flag == 0
            newJointTransform = eye(4);
            newJointTransform(3,4) = 0.1;       % hand is 0.1m above arm eef
            setFixedTransform(newBody.Joint, newJointTransform);
            hand_exist_flag = 1;
        end
        addBody(robot0, newBody, hand_name);
        hand_name = newBody.Name;
    end
    if handType(5)==1
        newBody = rigidBody('handYr');
        newBody.Joint = rigidBodyJoint('hand_jointYR','revolute');
        newBody.Joint.JointAxis = [0 1 0];
        newBody.Joint.PositionLimits = [-handRanges(5), handRanges(5)];
        if hand_exist_flag == 0
            newJointTransform = eye(4);
            newJointTransform(3,4) = 0.1;       % hand is 0.1m above arm eef
            setFixedTransform(newBody.Joint, newJointTransform);
            hand_exist_flag = 1;
        end
        addBody(robot0, newBody, hand_name);
        hand_name = newBody.Name;
    end
    if handType(6)==1
        newBody = rigidBody('handZr');
        newBody.Joint = rigidBodyJoint('hand_jointZR','revolute');
        newBody.Joint.JointAxis = [0 0 1];
        newBody.Joint.PositionLimits = [-handRanges(6), handRanges(6)];
        if hand_exist_flag == 0
            newJointTransform = eye(4);
            newJointTransform(3,4) = 0.1;       % hand is 0.1m above arm eef
            setFixedTransform(newBody.Joint, newJointTransform);
        end
        addBody(robot0, newBody, hand_name);
        hand_name = newBody.Name;
    end

    robot = copy(robot0);
    robot.Gravity = [0 0 -9.81];        % need for torque calculations

    env = {collisionBox(0.3, 0.3, 0.01)};   % default collision object (platform under robot)
    env{1}.Pose(3, end) = -0.01;
    show(robot, "Visuals","on", "Collisions","off");
    hold on
    show(env{1});

end