function [ m ] = motorMass( T_req, P_elec_one_motor, num_prop )
%% Description
% This code takes in the requirements of the motor and estimates the mass of all four.
%% Inputs
% T_req            - Maximum required torque of the motors [Nm]
% P_elec_one_motor - Required electrical power per motor [W]
% num_prop         - Number of motors/propellers [-]

%% Outputs
% m                - Mass of all motors [kg]

%---Begin Code---

%% Method #1
% From "Predicting Motor and Generator Maximum Torque as a Function of
% Mass", Without knowing the required T, the accuracy of this estimation is
% unclear.

MNT = 0.02;  % Mass Normalized Torque
alpha = 1.5;

m_one_motor = (T_req / MNT) ^ (1 / alpha);
m = num_prop * m_one_motor;

%% Method #2
% Infers the mass of the motors based on Rimfire Brushless Outrunner Motor
% Specifications.
if  P_elec_one_motor < 1800   
    m_motor = (P_elec_one_motor*7e-5 + 0.2135) * 2;   % Rough correlation for motor mass 
elseif  P_elec_one_motor >= 1800 && P_elec_one_motor < 2500
    m_motor = 0.634;% 2500 W motor mass
elseif  P_elec_one_motor >= 2500 && P_elec_one_motor < 5000    
    m_motor = 1.25; % 5000 W motor mass
elseif  P_elec_one_motor >= 5000 && P_elec_one_motor < 6500 
    m_motor = 1.48; % 6500 W motor mass
elseif  P_elec_one_motor >= 6500 
    warning('Can not find a RimFire motor to produce this much power! Motor mass is estimated from rough correlation.')
    m_motor = (P_elec_one_motor*7e-5 + 0.2135) * 2;   % Rough correlation for motor mass 
end  

m = m_motor * numProp;

%---End Code---

end
