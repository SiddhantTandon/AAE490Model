function [area_pan, m1_pan] = func_pan_p2m( p_charge, eff_pan, mars_flux, specPower_pan )
%func_pan_E2m 
%   This function calculates the area of the solar panels and the mass of
%   the panels. The inputs are the total battery energy, the efficiency of
%   the solar panels, the solar flux on mars, the time spent on the ground,
%   and the specific power of the solar panels.
area_pan = p_charge / (eff_pan * mars_flux);
m1_pan = (eff_pan * area_pan * mars_flux) / specPower_pan;
end

