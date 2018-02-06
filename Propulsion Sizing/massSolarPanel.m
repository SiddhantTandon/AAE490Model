function [ m_panel ] = massSolarPanel( P_r, solar_flux )
% 
% Description: 
%   Calculate a mass estimate for solar panels based on required power output
% 
% Inputs: 
%   P_r - Required Power [W]
%   solar_flux - Average solar flux on Mars surface (assumed constant) [W/m^2]      
% 
% Outputs:
%   m_panel - Mass estimation of solar panel [kg]
% 
% ASSUMPTIONS:
%   Constant solar flux on surface of Mars 
%       

% Constants
  e = 0.275;                   % Efficiency factor
  rp = 0.60;                   % Performance ratio
  cell_m_to_A_ratio = 0.84;    % Cell area to mass ratio [kg/m^2]
  hardware_factor = 1.05;      % Increase mass estimate by a fixed percentage to account for attachment hardware


% Calculations
  m_panel =(P_r * cell_m_to_A_ratio)/(e * solar_flux * rp);
  m_panel = hardware_factor * m_panel;     

end

