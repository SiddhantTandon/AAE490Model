%% AAE 490: Multidisciplinary Design
% Parametric Model 12
% 9 NOV 2017

% Revised:
% 27 JAN 2018

clear
clc

% Failure conditions
maxIter = 100;                      % Divergence criterion
maxMass = 900;                      % Max total system mass [kg]
maxArea = 5;                        % Max solar panel area [m^2]
maxVel = 67;                        % Max cruise speed [m/s]
minCruise = 60;                     % Min cruise time [s]


%% Constants

g = 3.71;                           % Gravitational acceleration [m/s^2] 
mission_R = 25 * 1000;              % Mission area radius [m]
mission_dur = 180;                   % Mission duration [sols]
sol_t = 24.7 * 3600;                % Length of sol [s/sol]
mars_flux = 190;                    % solar flux on mars [W/m^2]
image_fov = 30;                     % Field of view of camera (left to right) [deg]
rho_0 = 0.0139;                     % Atmospheric density at 500m [kg/m^3]
visc_dyn = 1.422*(10^-5);           % Dynamic viscosity [kg/ms]
rho_C = 1600;                       % Carbon fiber density [kg/m^3]
vtol_v = 2;                         % Vertical velocity during take-off and landing [m/s]
Rmv = 42.49;                        % Mass-to-volume ratio of ARES
                                    %       (used for approximating volume) [kg/m^3]
frame_rat = 0;%3333;                 % Airframe ratio of total mass [kg/kg]
c_D_body = 0.84;                    % Drag coefficient on body (bluff approximation) 
conv_toler = 0.01;                  % Convergence tolerance for change [%]

m_fix = 3;                          % Input total mass of payload, flight computer, and coms (Expected: 2-3kg)
p_pay = 25;                         % Input power requirement of the payload in operation load (Expected: 25W)
p_GNC = 28;                         % Input power requirement of onboard computers
p_comm = 1;                         % Input power requirement of communication systems

comm_percent = 0.0054;              % Input percent of time communicating per sol     

c_L = 1;                            % Input coefficient of 3D wing lift
LoD = 20;                           % Input lift over drag ratio of airfoil
wing_eff = 0.7;                     % Input efficiency factor of wing
wing_AR = 10;                       % Input aspect ratio of wing
eff_pan = 0.295;                    % Input efficiency of solar panels [%] (Expected: 29.5%)
specPower_pan = 60;                 % Input specific power of solar panels [W/kg] (Expected: 60 W/kg)
specEnergy_bat = 140;               % Input specific energy of batteries [Wh/kg] (Expected: 140 Wh/kg)

thr_FoS = 2.0;                      % Factor of safety for thrust
bat_FoS = 1.0;                      % Factor of safety for battery capacity
pan_FoS = 1.0;                      % Factor of safety for solar panel area

% Derived Constants
mission_A = pi * (mission_R ^2);	% Mission area [m^2]
comm_t = comm_percent * sol_t;      % Calculate time used to send/receive information per sol [s]
[cl_v_Re_coefs, maxRe] = BestFit_cl_v_Re();


%% Inputs

Inputs_h_cruise = 250:50:350;           % Cruise altitude [m]
Inputs_fly_t = (5:15) * 60;             % Input flight time per sol [s/sol]
Inputs_n_drone = 1:20;                  % Input number of drones
Inputs_wing_percent_lift = [0];         % Input percent of cruising lift obtained from wings
Inputs_prop_n_ver = [2,4,6];              % Input number of vertical propulsion motors
Inputs_propVer = [1];                 % Input vertical propulsion type (1 for propeller, 2 for ducted fan)
Inputs_prop_r_ver = 0.1:0.1:1.5;        % Input radius of vertical propulsion system blades
Inputs_prop_n_hor = [0];                % Input number of horizontal propulsion motors
Inputs_propHor = [1,2];                 % Input horizontal propulsion type (1 for propeller, 2 for ducted fan)
Inputs_prop_r_hor = 0.5:0.1:1;          % Input radius of horizontal propulsion system blades

% Instance storage
data_store = [];
fail_check = [0,0,0,0,0,0,0,0];     % Vertical, Horizontal, Both, Solar panel, Mass, Convergence, Cruise velocity, Cruise time

%% Primary Analysis Loop
fprintf('Analysis start:\n');
for h_cruise = Inputs_h_cruise
    vtol_t = h_cruise / vtol_v;         % Calculate time spent climbing and descending from flight altitude [s]
    image_w = 2 * h_cruise * tan(image_fov * pi / 180 / 2); % Field of View width [m]
    fprintf('    Cruising altitude: %d m\n', h_cruise);
    for fly_t = Inputs_fly_t
        cruise_t = fly_t - (2 * vtol_t);    % Calculate time available for survey per sol [s/sol]
        fprintf('        Flight time: %d s\n', fly_t);
        if cruise_t > minCruise             % Check success
            for n_drone = Inputs_n_drone
                day_A =  mission_A / mission_dur / n_drone;	% Calculate area that must be covered per sol [m^2/sol]
                vel_min = day_A / (cruise_t * image_w);    % Calculate minimum velocity at cruise [m/s]
%                 vel_min = 7.5;
                if vel_min < maxVel                 % Check success
                    for wing_percent_lift = Inputs_wing_percent_lift
                        for prop_n_ver = Inputs_prop_n_ver
                            for propVer = Inputs_propVer
                                for prop_r_ver = Inputs_prop_r_ver
                                    track_hor = 0;
                                    for prop_n_hor = Inputs_prop_n_hor
                                        for propHor = Inputs_propHor
                                            if prop_n_hor == 0
                                                propHor = 0;
                                            end
                                            for prop_r_hor = Inputs_prop_r_hor
                                                if prop_n_hor == 0
                                                    prop_r_hor = 0;
                                                    track_hor = track_hor + 1;
                                                end
                                                
                                                if track_hor > 1
                                                    break
                                                end
                                                %% Initial Conditions 
                                                m0_sys = m_fix;         % Initial system mass = mass of fixed material [kg]
                                                conv = conv_toler + 1;  % Reset the convergence value above 10%
                                                m0_ver = 0;             % Previous iteration vertical propulsion mass approximation [kg]
                                                m0_hor = 0;             % Previous iteration horizontal propulsion mass approximation [kg]
                                                m0_bat = 0;             % Previous iteration battery mass approximation [kg]
                                                m0_pan = 0;             % Previous iteration solar panel mass [kg]
                                                m0_wing = 0;            % Previous iteration wing mass [kg]
                                                m0_frame = 0;           % Previous iteration airframe [kg]
                                                n = 0;                  % iteration counter
                                                
                                                % Instance lists
                                                conv_list = [];
                                                m_sys_list = [m0_sys];
                                                m_ver_list = [m0_ver];
                                                m_hor_list = [m0_hor];
                                                m_bat_list = [m0_bat];
                                                m_pan_list = [m0_pan];
                                                m_wing_list = [m0_wing];
                                                m_frame_list = [m0_frame];
                                                n_list(1) = [0];
                                                
                                                %% Construct drone
                                                while conv > conv_toler

                                                    % Calculates the thrust/mass/power needed from vtol 
                                                    thr_vtol = g * m0_sys;			% Calculate thrust needed to vertically lift initial mass [N]
                                                    switch propVer
                                                        case 1
                                                            [~, m1_ver, success_ver] = ...
                                                                func_propeller((thr_vtol * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                            [p_vtol_ver, ~, ~] = ...
                                                                func_propeller(thr_vtol, rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                        case 2
                                                            [~, m1_ver, success_ver] = ...
                                                                func_ductfan((thr_vtol * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                            [p_vtol_ver, ~, ~] = ...
                                                                func_ductfan(thr_vtol, rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                    end
                                                    if ~success_ver                 % Mission failure
%                                                         fprintf('Failure: Vertical propulsion system blades exceed maximum wingtip speed\n');
                                                        fail_check(1) = fail_check(1) + 1;
                                                        break 
                                                    end
                                                    m1_sys = m0_sys - m0_ver + m1_ver; % Update the system mass [kg]


                                                    % Calculates the horizontal flight information
                                                    lift = m1_sys * g;
                                                    L_wing = lift * wing_percent_lift;                      % Calculate of lift required from wing
                                                    area_wing = L_wing / (0.5 * rho_0 * vel_min^2 * c_L);   % Calculate wing area
                                                    c_Di = c_L^2 / (pi * wing_AR * wing_eff);               % Calculate induced drag coefficient
                                                    c_D0 = lift / LoD;                                      % Calculate form drag coefficient
                                                    c_D_wing = c_Di + c_D0;                                 % Calculate drag coefficient of wing
                                                    D_wing = 0.5 * rho_0 * vel_min^2 * c_D_wing * area_wing;    % Calculate drag on wing
                                                    D_body = 0.5 * rho_0 * vel_min^2 * c_D_body * (m1_sys / Rmv)^(2/3);	% Calculate drag on body
                                                    m1_wing = 0.0254 * area_wing * rho_C;                   % Approximate wing mass
                                                    m1_sys = m1_sys + m1_wing - m0_wing;                    % Update system mass


                                                    % Calculate horizontal propulsion system mass and cruise power consumptions
                                                    thr_cruise_hor = D_wing + D_body;           % Calculate thrust required for unaccel., level flight (cruise)
                                                    thr_cruise_ver = m1_sys * g - L_wing;		  % Calculate remaining demand for vertical thrust
                                                    if prop_n_hor
                                                        success_ver = 1;
                                                        switch propVer
                                                            case 1
                                                                [~, m2_ver, success_ver1] = ...
                                                                    func_propeller((thr_cruise_ver * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                                if m2_ver > m1_ver
                                                                    m1_ver = m2_ver;
                                                                end
                                                                if ~success_ver1
                                                                    success_ver = success_ver1;
                                                                end
                                                                [p_cruise_ver, ~, ~] = ...
                                                                    func_propeller(thr_cruise_ver, rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                            case 2
                                                                [~, m2_ver, success_ver1] = ...
                                                                    func_ductfan((thr_cruise_ver * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                                if m2_ver > m1_ver
                                                                    m1_ver = m2_ver;
                                                                end
                                                                if ~success_ver1
                                                                    success_ver = success_ver1;
                                                                end
                                                                [p_cruise_ver, ~, ~] = ...
                                                                    func_ductfan(thr_cruise_ver, rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                        end
                                                        
                                                        switch propHor
                                                            case 1
                                                                [~, m1_hor, success_hor] = ...
                                                                    func_propeller((thr_cruise_hor * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_hor, prop_r_hor, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                                [p_cruise_hor, ~, ~] = ...
                                                                    func_propeller(thr_cruise_hor, rho_0, visc_dyn, rho_C, prop_n_hor, prop_r_hor, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                                m1_sys = m1_sys - m0_hor + m1_hor;		% Update the system mass [kg]
                                                                p_cruise_net = p_cruise_hor + p_cruise_ver;
                                                                
                                                            case 2
                                                                [~, m1_hor, success_hor] = ...
                                                                    func_ductfan((thr_cruise_hor * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_hor, prop_r_hor, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                                [p_cruise_hor, ~, ~] = ...
                                                                    func_ductfan(thr_cruise_hor, rho_0, visc_dyn, rho_C, prop_n_hor, prop_r_hor, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                                m1_sys = m1_sys - m0_hor + m1_hor;		% Update the system mass [kg]
                                                                p_cruise_net = p_cruise_hor + p_cruise_ver;
                                                        end
                                                    else
                                                        success_hor = 1;
                                                        thr_cruise_net = (thr_cruise_hor ^2 + thr_cruise_ver ^2) ^ 0.5;     % Calculate total thrust required at cruise
                                                        switch propVer
                                                            case 1
                                                                [~, m2_ver, success_ver1] = ...
                                                                    func_propeller((thr_cruise_net * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                                if m2_ver > m1_ver
                                                                    m1_ver = m2_ver;
                                                                end
                                                                if ~success_ver1
                                                                    success_ver = success_ver1;
                                                                end
                                                                [p_cruise_ver, ~, ~] = ...
                                                                    func_propeller(thr_cruise_net, rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                            case 2
                                                                [~, m2_ver, success_ver1] = ...
                                                                    func_ductfan((thr_cruise_net * thr_FoS), rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to mass conversion [kg]
                                                                if m2_ver > m1_ver
                                                                    m1_ver = m2_ver;
                                                                end
                                                                if ~success_ver1
                                                                    success_ver = success_ver1;
                                                                end
                                                                [p_cruise_ver, ~, ~] = ...
                                                                    func_ductfan(thr_cruise_net, rho_0, visc_dyn, rho_C, prop_n_ver, prop_r_ver, cl_v_Re_coefs, maxRe);	% Thrust to power conversion [kg]
                                                        end
                                                        m1_sys = m1_sys - m1_ver + m2_ver;		% Update the system mass [kg]
                                                        m1_ver = m2_ver;
                                                        m1_hor = 0;
                                                        p_cruise_net = p_cruise_ver;    
                                                    end
                                                    if ~success_hor && ~success_ver             % Mission failure
%                                                         fprintf('Failure: Propulsion system blades exceed maximum wingtip speed\n');
                                                        fail_check(3) = fail_check(3) + 1;
                                                        break
                                                    elseif ~success_hor                         % Mission failure
%                                                         fprintf('Failure: Horizontal propulsion system blades exceed maximum wingtip speed\n');
                                                        fail_check(2) = fail_check(2) + 1;
                                                        break
                                                    elseif ~success_ver                         % Mission failure
%                                                         fprintf('Failure: Vertical propulsion system blades exceed maximum wingtip speed\n');
                                                        fail_check(1) = fail_check(1) + 1;
                                                        break
                                                    end

                                                    % Energy needed for flight
                                                    E_cruise_net = p_cruise_net * cruise_t;
                                                    E_vtol_ver = p_vtol_ver * vtol_t;
                                                    fly_E = 2 * (E_vtol_ver) + E_cruise_net;

                                                    % Calculate total power requirements per flight and for battery sizing
                                                    total_flight_E = (fly_E + p_pay * fly_t + p_GNC * fly_t);                   % Total energy required per flight 
                                                    total_bat_E = (total_flight_E + p_GNC * (sol_t - fly_t) + p_comm * comm_t) * bat_FoS;   % Total energy of battery per day [J/sol]

                                                    % Calculate the mass of batteries required
                                                    m1_bat = func_bat_E2m(total_bat_E , specEnergy_bat);                        % Energy to mass conversion for battery mass [kg]
                                                    m1_sys = m1_sys + m1_bat - m0_bat;                                          % Update system mass [kg]

                                                    % Calculates the solar panel information
                                                    ground_t = sol_t / 2 - fly_t;                                                       % Amount of time spent on ground during the day [s]
                                                    p_charge = total_bat_E / ground_t;                                                  % Charging power required to charge battery fully in single sol[W]
                                                    [area_pan, m1_pan] = func_pan_p2m(p_charge, eff_pan, mars_flux, specPower_pan, pan_FoS);		% Calculate mass of solar panel [kg]
                                                    m1_sys = m1_sys + m1_pan - m0_pan;                                                  % Update system mass [kg]
                                                    if (area_pan > maxArea)                                                             % Mission failure
%                                                         fprintf('Failure: Solar panel area exceeds acceptable value\n');
                                                        fail_check(4) = fail_check(4) + 1;
                                                        break
                                                    end
                                                    
                                                    % Calculates airframe
                                                    m1_frame = (m1_sys - m0_frame) * frame_rat / (1 - frame_rat);
                                                    m1_sys = m1_sys + m1_frame;

                                                    % Resets all old mass values with the new values	
                                                    conv = (m1_sys / m0_sys) - 1;	% Calculate increase in mass since previous iteration
                                                    m0_sys = m1_sys;
                                                    m0_ver = m1_ver;
                                                    m0_hor = m1_hor;
                                                    m0_bat = m1_bat;
                                                    m0_pan = m1_pan;
                                                    m0_wing = m1_wing;
                                                    m0_frame = m1_frame;

                                                    n = n + 1;

                                                    if ((m0_sys * n_drone) > maxMass)       % Mission failure
%                                                         fprintf('Failure: Maximum mass reached\n');
                                                        fail_check(5) = fail_check(5) + 1;
                                                        break
                                                    end

                                                    % Saves masses, powers, and convergence factor in array to track
                                                    conv_list(n + 1) = conv;
                                                    m_sys_list(n + 1) = m1_sys;
                                                    m_ver_list(n + 1) = m1_ver;
                                                    m_hor_list(n + 1) = m1_hor;
                                                    m_bat_list(n + 1) = m1_bat;
                                                    m_pan_list(n + 1) = m1_pan;
                                                    m_wing_list(n + 1) = m1_wing;
                                                    m_frame_list(n + 1) = m1_frame;
                                                    n_list(n + 1) = n;
                                                end
                                                
                                                %% Check success and store
                                                if (n > maxIter)
%                                                     fprintf('Failure: Convergence failure\n');
                                                    fail_check(6) = fail_check(6) + 1;
                                                elseif success_ver && success_hor && (area_pan < maxArea) &&...
                                                        ((m0_sys * n_drone) < maxMass) && success_ver && success_hor
%                                                     fprintf('Mission success:\n');
                                                    
                                                    struct_store.n_drone = n_drone;
                                                    struct_store.fly_t = fly_t;
                                                    struct_store.h_cruise = h_cruise;
                                                    struct_store.propVer = propVer;
                                                    struct_store.propHor = propHor;
                                                    struct_store.prop_n_ver = prop_n_ver;
                                                    struct_store.prop_n_hor = prop_n_hor;
                                                    struct_store.prop_r_ver = prop_r_ver;
                                                    struct_store.prop_r_hor = prop_r_hor;
                                                    struct_store.wing_percent_lift = wing_percent_lift;

                                                    struct_store.m_sys = m0_sys;
                                                    struct_store.m_ver = m0_ver;
                                                    struct_store.m_hor = m0_hor;
                                                    struct_store.m_bat = m0_bat;
                                                    struct_store.m_pan = m0_pan;
                                                    struct_store.area_pan = area_pan;
                                                    struct_store.m_wing = m0_wing;
                                                    struct_store.m_frame = m0_frame;
                                                    struct_store.E_bat = total_bat_E;
                                                    struct_store.vel_min = vel_min;
                                                    struct_store.cruise_t = cruise_t;

%                                                     struct_store.n_list = n_list;
%                                                     struct_store.conv_list = conv_list;
%                                                     struct_store.m_sys_list = m_sys_list;
%                                                     struct_store.m_ver_list = m_ver_list;
%                                                     struct_store.m_hor_list = m_hor_list;
%                                                     struct_store.m_bat_list = m_bat_list;
%                                                     struct_store.m_pan_list = m_pan_list;
%                                                     struct_store.m_wing_list = m_wing_list;
%                                                     struct_store.m_frame_list = m_frame_list;

                                                    data_store = [data_store, struct_store];
                                                end
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                else
%                     fprintf('Failure: Cruising velocity exceeds maximum bound\n');
                    fail_check(7) = fail_check(7) + 1;
                end
            end
        else
%             fprintf('Failure: Not enough time spent in cruise\n');
            fail_check(8) = fail_check(8) + 1;
        end
    end
end

%% Score results
data_store = func_weight(data_store);


%% Output results

fprintf('Results:\n_____________________\n');
fail_count = sum(fail_check);
success_count = length(data_store);
fprintf('Total sets examined: %d\nMission successes detected: %d\nMission failures detected: %d\n', (fail_count + success_count) , success_count, fail_count);
if success_count
    high_score = -(10^6);
    for index = 1:length(data_store)
        if data_store(index).score > high_score
            high_score = data_store(index).score;
            best_index = index;
        end
    end
    highScore_data = data_store(best_index);
    fprintf('    Best scoring system operates with a total mass of %.2f kg\n', (highScore_data.n_drone * highScore_data.m_sys));
    fprintf('    Input conditions:\n        Number of drones: %d\n', highScore_data.n_drone);
    fprintf('        Daily flight time: %d s\n', highScore_data.fly_t);
    fprintf('        Cruising altitude: %d m\n', highScore_data.h_cruise);
    fprintf('        Number of vertical propulsion systems: %d\n', highScore_data.prop_n_ver);
    switch highScore_data.propVer
        case 1  
            fprintf('            Vertical propulsion type: Propeller\n');
        case 2
            fprintf('            Vertical propulsion type: Ducted fan\n');
    end
    fprintf('            Radius of vertical propulsion system blades: %.1f m\n', highScore_data.prop_r_ver);
    fprintf('        Number of horizontal propulsion systems: %d\n', highScore_data.prop_n_hor);
    if highScore_data.prop_n_hor
        switch highScore_data.propHor
            case 1  
                fprintf('            Horizontal propulsion type: Propeller\n');
            case 2
                fprintf('            Horizontal propulsion type: Ducted fan\n');
        end
        fprintf('            Radius of horizontal propulsion system blades: %.1f m\n', highScore_data.prop_r_hor);
    end
    fprintf('        Wing contribution to lift: %d\n', highScore_data.wing_percent_lift);
    
    fprintf('\n    Design:\n        Mass of single drone: %.2f kg\n', highScore_data.m_sys);
    fprintf('        Vertical propulsion system mass: %.2f kg\n', highScore_data.m_ver);
    if highScore_data.prop_n_hor
        fprintf('        Horizontal propulsion system mass: %.2f kg\n', highScore_data.m_hor);
    end
    fprintf('        Battery mass: %.2f kg\n', highScore_data.m_bat);
    fprintf('            Battery energy capacity: %d Wh\n', (highScore_data.E_bat/3600));
    fprintf('        Solar panel mass: %.2f kg\n', highScore_data.m_pan);
    fprintf('            Solar panel area: %.1f m^2\n', highScore_data.area_pan);
    if highScore_data.m_wing
        fprintf('        Wing mass: %.2f kg\n', highScore_data.m_wing);
    end
    fprintf('        Cruising velocity: %.1f m/s\n', highScore_data.vel_min);
    fprintf('        Cruise time: %d s\n', highScore_data.cruise_t);

%     fprintf('\n    Iteration lists:\n        Iteration index:');
%     disp(highScore_data.n_list);
%     fprintf('        Convergence percent:');
%     disp(highScore_data.conv_list);
%     fprintf('        Drone masses (kg):');
%     disp(highScore_data.m_sys_list);
%     fprintf('        Vertical propulsion masses (kg):');
%     disp(highScore_data.m_ver_list);
%     if highScore_data.prop_n_hor
%         fprintf('        Horizontal propulsion mass (kg):');
%         disp(highScore_data.m_hor_list);
%     end
%     fprintf('        Battery masses (kg):');
%     disp(highScore_data.m_bat_list);
%     fprintf('        Solar panel masses (kg):');
%     disp(highScore_data.m_pan_list);
%     if highScore_data.m_wing
%         fprintf('        Wing masses (kg):');
%         disp(highScore_data.m_wing_list);
%     end
end