function [ cap_batt ] = capacityBattery( P_draw, V_batt, t_run )
% 
% Description: 
%   Calculate a mass estimate for batteries based on required flight time,
%   motor power draw, and battery voltage
% 
% Inputs: 
%   P_draw - Power draw of entire electrical system during flight [W]  
%   V_batt - Battery voltage [V]
%   t_run  - Required Flight Time [hr]
% 
% Outputs:
%   cap_batt - Estimated battery capacity [A*hr]
% 
% ASSUMPTIONS:
%   Neglect the Peukart effect (small effect for Lithium Ion, Lithium
%      Polymer, NiCd and NiMH batteries)
%      [Source: https://www.powerstream.com/battery-capacity-calculations.htm]


% Constants
  discharge_percent = 0.7;  % Percentage of total battery capicity that is useful on each cycle (Lithium batteries should not be drained to 0%)  

% Calculations
  % t_run = t_run / 60;    % Convert time from mins to hours (IF INPUT IS IN MINUTES)

  cap_batt = P_draw/V_batt * t_run;    % [Amp * hr] = [W/V * hr]    (If the input is in Power draw)

  cap_batt = cap_batt/discharge_percent;     % Adjust total capacity to account for useful battery capacity

end
