function val = objFun(config)
    
     % % ALPHA weights
    w_i = [40 30 ...    % obj position, angle
            2 1 0.1 ...   % manip, joint lim, collide
            0.1 ...     % WIHM
            0.1 ...     % gravity torque
            2 3 4 ... % motion smoothness
            0.1];           % object jerk


    % BETA weights
%     w_i = [7 5 ...    % obj position, angle
%             2 1 0.1 ...   % manip, joint lim, collide
%             0.1 ...     % WIHM
%             0.1 ...     % gravity torque
%             6 5 4 ... % motion smoothness
%             2];           % object jerk

    para.n = 1;         % parameters for grooveLoss
    para.s = 0;
    para.c = 0.1;
    para.r = 7;

    global robstruct
    robstruct.w_i = w_i;
    robstruct.para = para;
    
    w_i = w_i./norm(w_i);

    val =  w_i(1)*grooveLoss(objFun_ObjectPosition(config), para) + ...
           w_i(2)*grooveLoss(objFun_ObjectRotation(config), para) + ...
           w_i(3)*grooveLoss(objFun_Manipulability(config), para) + ...
           w_i(4)*grooveLoss(objFun_JointLimits(config), para) + ... %            
           w_i(5)*grooveLoss(objFun_Collisions(config), para) + ...
           w_i(6)*grooveLoss(objFun_WIHM(config), para) + ...
           w_i(7)*grooveLoss(objFun_ArmGravityTorque(config), para) + ...
           w_i(8)*grooveLoss(objFun_ArmVelocity(config), para) + ...
           w_i(9)*grooveLoss(objFun_ArmAcceleration(config), para) + ...
           w_i(10)*grooveLoss(objFun_ArmJerk(config), para) +...
           w_i(11)*grooveLoss(objFun_ObjectJerk(config), para) ;
    
end


%{

w_i index dictionary:

1 - Object position 
2 - Object orientation

3 - Arm Singularity
4 - Arm Joint Limits
5 - Arm Collisions

6 - WIHM Motion required (Accuracy / Likelihood of dropping object)

7 - Arm Gravity Torques

8 - Arm Joint Velocity
9 - Arm Joint Acceleration
10- Arm Joint Jerk

11- Object Jerk

%}