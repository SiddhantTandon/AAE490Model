function [ rho ] = rhoMars( h )
% rhoMars: Calculate the atmospheric density on Mars given an altutude abve the reference plane
%   Inputs: 
%     h: altitude above the reference plane [m]
% 
%   Outputs:
%     rho: atmosphere density [kg/m^3]
% 
%    Source: https://www.grc.nasa.gov/www/k-12/airplane/atmosmrm.html

    assert(h>=0,'Mars altitude must be greater than or equal to 0 meters to calculate density')
    
    if h >= 0 && h <= 7000
        T = -31 - 0.000998 * h;
        p = .699 * exp(-0.00009 * h);
    elseif h > 7000
        T = -23.4 - 0.00222 * h;
        p = .699 * exp(-0.00009 * h);
    end

    rho = p / (0.1921 * (T + 273.1));

end

