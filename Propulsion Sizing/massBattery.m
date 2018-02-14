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
%   Battery experiences 0 degree C temperatures on average for mission lifespan
% 
% Notes:
%   Li-Ion loses 2% Columbic Efficiency per month at 0 degrees C --> ~6% for 90 sols; source: http://batteryuniversity.com/learn/article/bu_808b_what_causes_li_ion_to_die 
%   "Coulombic efficiency is capable of measuring both changes: the
%   lithium lost due to SEI growth on the anode and electrolyte oxidation at the cathode. The results can be used to rank the life expectancy of a battery by quantifying the parasitic reaction."
%     
% Design Variables
    specific_energy = 350;    % [W*hr/kg]   (Typical range for lithium batteries: 100-250 Wh/kg); 350 is extrapolated for the year 2030
    
    cap_batt_eff = cap_batt / 0.94;  % accounts for 6% Columbic efficiency lost during mission lifespan
    m_batt = cap_batt_eff * V_batt / specific_energy;
    
end

