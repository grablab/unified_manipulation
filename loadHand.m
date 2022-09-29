function [hand_type, hand_ranges, handDOFs] = loadHand(hand)
    
% % hand_type is boolean actuated hand dof at [X,Y,Z,XR,YR,ZR] respectively
% % hand_ranges specifed for the actuated hand DOFs (0 if not actuated)

     switch hand
         case 'stewart'
             hand_type = [1 1 1 1 1 1];
             hand_ranges = [0.025 0.025 0.025 deg2rad([30 30 90])];

         case 'modelO'  % estimated
             hand_type = [0 0 1 1 1 0];
             hand_ranges = [0 0 0.02 deg2rad([20 20 0])];
        
         case 'modelQ' % taken from RAL finger gaiting paper
             hand_type = [0 1 1 1 0 1];
             hand_ranges = [0 0.05 0.05 deg2rad([90 0 55])];

         case 'spherical'
             hand_type = [0 0 0 1 1 1];
             hand_ranges = [0 0 0 deg2rad([30 30 30])];

         case 'fixed' % e.g. parallel jaw
             hand_type = [1 0 0 0 0 0];
             hand_ranges = [1e-7 0 0 0 0 0];

         otherwise
             hand_type = [0 0 0 0 0 0];
             hand_ranges = [0 0 0 0 0 0];

     end
%  %    hand_ranges = hand_ranges(hand_type>0);   % storing hand_ranges only for actuated DOFs

     handDOFs = sum(hand_type>0);
end