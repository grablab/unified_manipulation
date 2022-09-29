function cost_arr = calcObjFunCosts(prevCfg_all, objPoses_arr)
    global robstruct

    prevCfg_all = prevCfg_all(end:-1:1, :);
    nPoints = size(objPoses_arr.xyz,1);
    cost_arr = zeros(nPoints, 11);

    for oo=1:nPoints
        config = prevCfg_all(oo+4,:);
        robstruct.goal_xyz = objPoses_arr.xyz(oo, :);
        robstruct.goal_R = eul2rotm(objPoses_arr.ang(oo, :), 'XYZ');

        robstruct.prevCfg = prevCfg_all(oo:oo+3,:);
        
        cost_arr(oo, 1)  = objFun_ObjectPosition(config);
        cost_arr(oo, 2)  = objFun_ObjectRotation(config);
        cost_arr(oo, 3)  = objFun_Manipulability(config);
        cost_arr(oo, 4)  = objFun_JointLimits(config);
        cost_arr(oo, 5)  = objFun_Collisions(config);
        cost_arr(oo, 6)  = objFun_WIHM(config);
        cost_arr(oo, 7)  = objFun_ArmGravityTorque(config);
        cost_arr(oo, 8)  = objFun_ArmVelocity(config);
        cost_arr(oo, 9)  = objFun_ArmAcceleration(config);
        cost_arr(oo, 10) = objFun_ArmJerk(config);
        cost_arr(oo, 11) = objFun_ObjectJerk(config);
    end

end