function [ area_panel, m_panel ] = sizeSolarPanel( P_r, solar_flux )
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
  e = 0.35;                    % Efficiency factor
  rp = 0.85;                   % Performance ratio
  cell_m_to_A_ratio = 0.84;    % Cell area to mass ratio [kg/m^2]


% Calculations
  area_panel = (P_r)/(e * solar_flux * rp);

  m_panel = area_panel * cell_m_to_A_ratio;

end

