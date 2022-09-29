function val = normalize(val, mean, std)
    z = 1.6;        % for 90 percent data
    
    lo = max(mean-z*std, 0);
    hi = min(mean+z*std, 1);

    val = min(max(val - lo, 0)/(hi - lo), 1);
    
end