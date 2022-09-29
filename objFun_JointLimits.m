function val = objFun_JointLimits(config)
    global robstruct
    
    lower_lim = robstruct.armJntLim_lo;
    upper_lim = robstruct.armJntLim_up;

    penFactor_k = 8e6;

    armCfg = config(1:end-robstruct.handDOFs);

    exponent_term = ((armCfg - lower_lim).*(upper_lim - armCfg)) ./ ...
            ((upper_lim - lower_lim).^2);

    val = exp(-penFactor_k*prod(exponent_term));        % removed the "1-exp" to minimize

    % From sampled configuraitons offline
    switch robstruct.arm_eef_name
        case 'iiwa_link_ee_kuka'        % KUKA
            mean_jointlim = 0.4339;
            std_jointlim = 0.2425;   
        case 'tool0'                    % UR10
            mean_jointlim = 0.3316;
            std_jointlim = 0.1207;
    end

    val = normalize(val, mean_jointlim, std_jointlim);


end