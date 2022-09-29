function val = objFun_ObjectPosition(config)
    global robstruct

    goal_xyz = robstruct.goal_xyz;
    obj_xyz = tform2trvec(getTransform(robstruct.robot, config, robstruct.hand_name));

    val = norm(goal_xyz - obj_xyz);

end