function val = objFun_ArmVelocity(config)
    global robstruct

    armCfg = config(1:end-robstruct.handDOFs);
    prevCfg = robstruct.prevCfg(:,1:end-robstruct.handDOFs);  % only considering the arm joints
    h = robstruct.h;

    val = (3*prevCfg(4,:) - 16*prevCfg(3,:) + 36*prevCfg(2,:) - 48*prevCfg(1,:) + 25*armCfg)/(12*h);
    
    val = val./robstruct.max_vel;
    
    val = min(norm(val, 20), 1);

end