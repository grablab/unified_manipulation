function val = objFun_ArmJerk(config)
    global robstruct

    armCfg = config(1:end-robstruct.handDOFs);
    prevCfg = robstruct.prevCfg(:,1:end-robstruct.handDOFs);  % only considering the arm joints
    h = robstruct.h;

    val = (3*prevCfg(4,:) - 14*prevCfg(3,:) + 24*prevCfg(2,:) - 18*prevCfg(1,:) + 5*armCfg)/(2*h^3);

    val = val./robstruct.max_jerk;
    
    val = min(norm(val, 20), 1);

end