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
% Notes:
%   maxI should be > Idraw

% Constants
  discharge_percent = 0.7;  % Percentage of total battery capicity that is useful on each cycle (Lithium batteries should not be drained to 0%)  

% Calculations
    % t_run = t_run / 60;    % Convert time from mins to hours (IF INPUT IS IN MINUTES)

    cap_batt_nom = (P_draw/V_batt) * t_run;    % [Amp * hr] = [W/V * hr]    (If the input is in Power draw)
    
    C = 1 / t_run;    %C rating = C/t_discharge, source: http://www.ecochemie.nl/download/Applicationnotes/Autolab_Application_Note_BAT02.pdf, and https://web.statler.wvu.edu/~wu/mae493/7-storage-2.pdf    
    maxI = cap_batt_nom * C;  %max current the battery can produce
    Idraw = P_draw / V_batt;  %current the battery is required to produce
    
    cap_batt = cap_batt_nom / discharge_percent;     % Adjust total capacity to account for useful battery capacity
    
end

