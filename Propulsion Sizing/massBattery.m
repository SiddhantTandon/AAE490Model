function [ m_batt ] = massBattery( cap_batt, V_batt )
% 
% Description: 
%   Calculate a mass estimate for solar panels based on required power output
% 
% Inputs: 
%   cap_batt - Estimated battery capacity [A*hr]
%   V_batt - Battery voltage [V]
% 
% Outputs:
%   m_batt - Mass estimation of all batteries [kg]
% 
% ASSUMPTIONS:
%   Use a simple estimation mass of battery based on typical specific energy of lithium batteries  
%   Use specific energy that is better than what is currently available to extrapolate available technology to the actual mission timeframe   
%     
% Design Variables
    specific_energy = 350;    % [W*hr/kg]   (Typical range for lithium batteries: 100-250 Wh/kg)
    
    m_batt = cap_batt * V_batt / specific_energy;
    
end

