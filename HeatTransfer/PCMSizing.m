clear
clc
close

%% Constants/Inputs

n = 2; %index of PCM material for analysis
SF = 1.2; %safety factor applied to final mass

% Motor Operational Specifications
T_motor_max = 110; %maximum allowable motor temperature [C] (30 for batteries)
T_motor_min = -20; %minimum allowable motor temperature (0 for batteries)
t = 10 * 60; %flight time [s]

% Environmental Constants
T_mars = 0; %ambient temperature on Mars [C]

% Motor Specifications
P_waste = 650; %Set power lost by 1 motor for consistancy [W] (200 for battieres)
E_waste = P_waste * t;

T_mars = T_mars + 273.15; %convert to K
T_motor_max = T_motor_max + 273.15; %convert to K
T_motor_min = T_motor_min + 273.15; %convert to K


%% Phase Change Materials
%{
Notes:
1) Sized assuming no convective or radiative heat transfer
2) Cell array structure: {material, T_melt, H_delta_fus, c_p_liquid, c_p_solid
                            rho_solid, rho_liquid, k}
3) List of phase change materials: https://en.wikipedia.org/wiki/Phase-change_material
4) Many tunable PCM materials available
5) Note: Inorganic material is 21 C Infinite R Energy Sheet
%}

% Material Properties
materialOptions = {'Water'                ,   0, 300000, 4187, 2108, 916 , 995 , 2   ;
                   '0400-Q20 BioPCM (Wax)',  20, 215000, 3200, 3500, 1075, 1125, 0.45;
                   '0100-Q50 BioPCM (Wax)', -50, 215000, 3200, 3500, 1075, 1075, 0.45;
                   %'0500-Q50 BioPCM (Wax)',  50, 215000, 3200, 3500, 1075, 1075, 0.45;
                   'Inorganic'            ,  21, 200000, 3140, 3140, 1540, 1540, 1.09};

PCM = materialOptions{n, 1}; %phase change material
T_melt = materialOptions{n, 2}; %melting point [C]
H_delta_fus = materialOptions{n, 3}; %latent heat of fusion [J/kg]
c_p_liquid = materialOptions{n, 4}; %heat capacity of liquid [J/kg*K]
c_p_solid = materialOptions{n, 5}; %heat capacity of solid [J/kg*K]
rho_solid = materialOptions{n, 6}; %density of solid [kg/m^3]
rho_liquid = materialOptions{n, 7}; %density of liquid at maximum temp [kg/m^3]

T_melt = T_melt + 273.15; %melting point [K]
T_0 = max(T_mars, T_motor_min);
T_f = T_motor_max;

% Energy absorbtion from heating solid
if T_melt > T_0
    Q_solid_1kg = c_p_solid * (T_melt - T_0);
else
    Q_solid_1kg = 0;
end

% Energy absorbtion from melting
if T_melt <= T_f
    Q_melt_1kg = H_delta_fus;
else
    Q_melt_1kg = 0;
end

% Energy abosrtion from heating liquid
if T_melt < T_f
    Q_liquid_1kg = c_p_liquid * (T_f - T_melt);
else
    Q_liquid_1kg = 0;
end

% Total energy absorbably in change from T_0 to T_f
c_p_total = Q_solid_1kg + Q_melt_1kg + Q_liquid_1kg; %total heat capacity [J/kg]

% Mass of system
m_PCM = SF * E_waste / c_p_total; %mass of PCM required to absorb heat [kg]
m_PCM_struc = m_PCM * 0.25; %estimate of additional structurla mass required [kg]

V_liquid = m_PCM / rho_liquid; %volume of liquid [m^3]
V_solid = m_PCM / rho_solid; %volume of solid [m^3]

percent_vol_change = (V_liquid - V_solid) / V_solid;

%% Plot of PCM temperature vs. time

[numMaterials, ~] = size(materialOptions);
T_max = 3000; %ridiculously high number to make sure sufficient energy is added
E_total = 500*1000;
T_plot = zeros(numMaterials, 4);
T_f = zeros(numMaterials, 1);
E_plot = zeros(numMaterials, 4);

for k = [1:numMaterials]
    
    T_melt = materialOptions{k, 2} + 273.15; %melting point [K]
    H_delta_fus = materialOptions{k, 3}; %latent heat of fusion [J/kg]
    c_p_liquid = materialOptions{k, 4}; %heat capacity of liquid [J/kg*K]
    c_p_solid = materialOptions{k, 5}; %heat capacity of solid [J/kg*K]
        
    
    % Initial temperature at or below melting point
    if T_0 <= T_melt
        Q_solid_1kg = c_p_solid * (T_melt - T_0);
        Q_melt_1kg = H_delta_fus;
        Q_liquid_1kg = c_p_liquid * (T_max - T_melt);
        T_plot(k, :) = [T_0, T_melt, T_melt, T_max]; %temperature [K]
        
    % Initial Temperature above melting point
    else %(T_0 > T_melt)
        Q_solid_1kg = 0;
        Q_melt_1kg = 0;
        Q_liquid_1kg = c_p_liquid * (T_max - T_melt);
        T_plot(k, :) = [T_0, T_0, T_0, T_max]; %temperature [K]
    end
    
    if Q_solid_1kg >= E_total
        T_f(k) = T_0 + E_total / c_p_solid;
    elseif (Q_solid_1kg + Q_melt_1kg) >= E_total
        T_f(k) = T_melt;
    elseif (Q_solid_1kg + Q_melt_1kg + Q_liquid_1kg) >= E_total
        T_f(k) = T_0 + (E_total - Q_solid_1kg - Q_melt_1kg) / c_p_liquid;
    else
        fprintf('Increase Final Temperature\n')
    end
    
    E_plot(k, :) = [0, Q_solid_1kg, Q_solid_1kg+Q_melt_1kg, Q_solid_1kg+Q_melt_1kg+Q_liquid_1kg]; %energy [J]
end

figure(1)
h = axes('Position', [0.13, 0.2, 0.7750, 0.7250]);
plot(E_plot(1, :) / 1000, T_plot(1, :) - 273.15, '-')
hold on
for l = 2:numMaterials
    plot(E_plot(l, :) / 1000, T_plot(l, :) - 273.15, '-')
end
hold off

axis([0, E_total/1000, T_0 * 0.9 - 273.15, max(T_f) * 1.1-273.15])

xlabel('Energy Added per Unit Mass [kJ/kg]');
ylabel('PCM Temperature [C]');
title('Phase Change Diagram');
legend(materialOptions{:, 1});

% Annotations
annotation('arrow', [0.55, 0.65], [0.05, 0.05])
annotation('textbox', [0.4, 0.025, 0.3, 0.05], 'EdgeColor', 'none',...
           'FontWeight', 'bold', 'string', {'Flight Time'});
annotation('textbox', [0.7, 0.3, 0.3, 0.05], 'EdgeColor', 'none', 'string', {'Water'});
annotation('textbox', [0.7, 0.4, 0.3, 0.05], 'EdgeColor', 'none', 'string', {'Wax 04'});
annotation('textbox', [0.7, 0.5, 0.3, 0.05], 'EdgeColor', 'none', 'string', {'Inorganic'});
annotation('textbox', [0.7, 0.7, 0.3, 0.05], 'EdgeColor', 'none', 'string', {'Wax 01'});

%% Results

fprintf('Material: %s\n', PCM);
fprintf('Mass of PCM: %.2f kg\n', m_PCM);
fprintf('Approximate Mass of Additional Structure: %.2f kg\n', m_PCM_struc);
fprintf('Approximate Total Mass: %.2f kg\n', m_PCM_struc + m_PCM);
if V_liquid > V_solid
    fprintf('Maximum Volume of PCM: %.2f [L] (liquid)\n', V_liquid * 1000)
else 
    fprintf('Maximum Volume of PCM: %.2f [L] (solid)\n', V_solid * 1000)
end
fprintf('Percentage change in volume: %.2f%%\n', percent_vol_change*100)