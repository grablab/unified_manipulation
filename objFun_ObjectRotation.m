function val = objFun_ObjectRotation(config)
    global robstruct

    goal_R = robstruct.goal_R;
    obj_R = tform2rotm(getTransform(robstruct.robot, config, robstruct.hand_name));

    Rdiff = obj_R*goal_R';

    val = norm(abs(rotm2eul(Rdiff, 'XYZ')));

end