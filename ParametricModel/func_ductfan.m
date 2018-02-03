function [P_total , M_total, reasonableSln] = func_ductfan(T, rho, visc, rho_cf, num_prop, R, cl_v_Re_coefs, maxRe)
% Ducted Fan code for calculating duct mass

%Ducted fan allows higher tip speed, so the reasonableSln boolean may
%trigger incorrectly.  To avoid this, the amount of thrust required is
%reduced, so for a given radius propeller, the motor size and required spin
%rate is decreased, but the overall thrust produced (due to the duct) does
%not change.
T_equi_prop = T / 1.94;

[P_total, M_total_noDuct, reasonableSln] = func_propeller(T_equi_prop, rho, visc, rho_cf, num_prop, R, cl_v_Re_coefs, maxRe);

% Duct Dimensions
depth_duct = R;
t_duct = 0.02 * R;

% Add effects of duct
Mass_1_duct = 2 * pi * R * t_duct * depth_duct * rho_cf;
M_total = M_total_noDuct + Mass_1_duct * num_prop; % Total mass of the duct
% total power required is corrected for by differen
end
