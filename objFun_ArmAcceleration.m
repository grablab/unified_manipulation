function val = objFun_ArmAcceleration(config)
    global robstruct

    armCfg = config(1:end-robstruct.handDOFs);
    prevCfg = robstruct.prevCfg(:,1:end-robstruct.handDOFs);  % only considering the arm joints
    h = robstruct.h;

    val = (11*prevCfg(4,:) - 56*prevCfg(3,:) + 114*prevCfg(2,:) - 104*prevCfg(1,:) + 35*armCfg)/(12*h^2);

    val = val./robstruct.max_acc;

    val = min(norm(val, 20), 1);

end