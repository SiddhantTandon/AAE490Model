function [m1_bat] = func_bat_E2m( total_bat_E , specEnergy_bat )
%func_bat_E2m 
%   Function calculates the mass of batteries given the total energy
%   required for the batteries and the specific energy density of the
%   batteries.
    m1_bat = total_bat_E /(3600 * specEnergy_bat); %[kg]
end

