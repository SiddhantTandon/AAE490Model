function [ volume_batt, num_series, num_parallel2, total_cells, R ] = voluBattery(V_batt, energy_batt)
%
% Description: 
%   Calculate the volume of the resultant battery system based on the
%   previously calculated capacity
%
% Inputs:
%   V_batt - voltage of the battery 
%   V_motor - Voltage of the motor
%   energy_batt - Total energy required by battery (includes the 70% usable capacity factor) [W*hr]
%
% Outputs:
%   volume_batt - volume of the resultant battery system
%   num_series - number of battery cells in series from resulting configuration
%   num_parallel - number of (battery cell blocks in series) in parallel from resulting configuration
%   total_cells - totall number of battery cells in resulting configuration
%   R - Internal resistance of total battery given internal resistance of one cell


cap_cell = 3; %AH
E_req = energy_batt * 3600; % [W]*[hr]* 3600[s/hr] = W*s = J

V_mean = 3.6; %[V]
num_series = ceil(V_batt/V_mean); %number of cells in series, round up
numJ = V_mean*num_series*cap_cell*3600; %[V]*[number in series]*[A*hr]*[s/hr] = [W*hr]*[s/hr] = [W*s] = [J]
num_parallel2 = ceil((E_req)/numJ);
total_cells = num_series*num_parallel2;

%Battery Resistance
Rcell = 0.003; %ohms, year 2030 extrapolation
Rseries = num_series*Rcell; %R of row in series
R = Rseries/num_parallel2; %All rows of cells have same Rseries, this is total R of battery

%For 25500 battery cells
%http://www.ebay.com/gds/A-Beginners-Guide-to-Buying-Li-Ion-Batteries-/10000000177590030/g.html 
cell_volume = pi*((0.0193/2)^2)*0.0492;
volume_batt = cell_volume*total_cells;
