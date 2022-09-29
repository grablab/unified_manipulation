function [sepDist] = filterSepDist(sepDist_raw)
    
    % ignore distances between bodies that start too close to each other (5-10cm)

    thresh = 0.08;
    
    sepDist_raw(sepDist_raw<thresh) = NaN;
    
    sepDist = sepDist_raw;

end