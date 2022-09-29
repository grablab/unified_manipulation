function out = grooveLoss(val, para)
    
    % Parametric normalization function from RelaxedIK
    n = para.n;
    s = para.s;
    c = para.c;
    r = para.r;

    out = ((-1)^n)*exp((-(val - s)^2)/(2*c^2)) + r*(val - s)^4;

end