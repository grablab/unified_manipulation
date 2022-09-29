function val = objFun_Manipulability(config)
    global robstruct
    
    jacob = geometricJacobian(robstruct.robot, config, robstruct.arm_eef_name);
    val = 1/cond(jacob);
    val = 1 - val;          % To minimize

    % From sampled configuraitons offline
    switch robstruct.arm_eef_name
        case 'iiwa_link_ee_kuka'        % KUKA
            mean_manip = 0.9328;
            std_manip = 0.0454;   
        case 'tool0'                    % UR10
            mean_manip = 0.9431;
            std_manip = 0.0540;
    end

    val = normalize(val, mean_manip, std_manip);

end