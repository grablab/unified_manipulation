function val = objFun_Collisions(config)
    global robstruct
    
    penFactor_a = 8;        % collisions

    [isCollide, sepDist] = checkCollision(robstruct.robot, config, robstruct.env);

    
    if any(isCollide)
        sepDist = 0;
    else
        sepDist = min(min(sepDist./robstruct.sepAtHome));
    end
    
    val = exp(-penFactor_a*sepDist);

    % From sampled configuraitons offline
    switch robstruct.arm_eef_name
        case 'iiwa_link_ee_kuka'        % KUKA
            mean_collide =  0.1144;
            std_collide = 0.2628;   
        case 'tool0'                    % UR10
            mean_collide =  0.2342;
            std_collide = 0.2628;
    end
    

    val = normalize(val, mean_collide, std_collide);

end




