function val = objFun_ObjectJerk(config)
    global robstruct

    prev_obj_xyz = zeros(4,3);
    h = robstruct.h;

    for rr = 1:4
        prev_obj_xyz(rr,:) = tform2trvec(getTransform(robstruct.robot, robstruct.prevCfg(rr,:),  ...
            robstruct.hand_name));
    end

    obj_xyz = tform2trvec(getTransform(robstruct.robot, config, robstruct.hand_name));

    val = (3*prev_obj_xyz(4,:) - 14*prev_obj_xyz(3,:) + 24*prev_obj_xyz(2,:) - 18*prev_obj_xyz(1,:) + 5*obj_xyz)/(2*h^3);

    val = norm(val, 20)./robstruct.max_obj_jerk;

    val = min(val, 1);

end