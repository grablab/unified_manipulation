function val = objFun_WIHM(config)
    global robstruct

    % required WIHM motion normalized by the hand range
    wihm_hand = config((end-robstruct.handDOFs+1):end)./robstruct.hand_ranges;

    val = norm(wihm_hand, 20);

end