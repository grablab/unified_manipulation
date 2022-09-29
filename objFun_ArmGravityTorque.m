function val = objFun_ArmGravityTorque(config)

    global robstruct

    joint_torques = inverseDynamics(robstruct.robot, config);   % in the presence of gravity
    % % Can add external force e.g. weight of added robot hand

    joint_torques = joint_torques(1:end-robstruct.handDOFs);

    val = norm(abs(joint_torques)./robstruct.max_torque, 20);

    val = min(val, 1);

end