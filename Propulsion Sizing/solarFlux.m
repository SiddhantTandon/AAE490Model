function [ flux_avg, day_length_hr ] = solarFlux(latitude, Ls)
% Description: 
%   Calculate a solar flux estimate [W/m^2] and length of sunlight duration 
%   in [hr] based on mission latitude and time of year
%
% Inputs:   
%    latitude: Geographic latitude of the mission on Mars Surface [deg] (Range between -90 and 90 deg) >> needs more testing on negative values
%    Ls (solar longitude): Angular position of Mars around the Sun [deg], based on time of year  (Range from 0 to 360 deg)
%                           -> Ls = 0° corresponds to northern vernal equinox
% Outputs: 
%    flux_avg: Average daily solar flux taken over only the daylight hours  [W/m^2]
%    day_length_hr: Length of daylight hours at the givin surface latitude [hr]
% 
% ASSUMPTIONS:
%   Constant optical depth 
%       Actual example optical depth variance data can be found online at https://ac.els-cdn.com/0038092X90901567/1-s2.0-0038092X90901567-main.pdf?_tid=e4601e00-1412-11e8-97ae-00000aacb35e&acdnat=1518893389_33db73dec0e65a7bb93a15b7ea47f75a
%   Calculations performed based on equations found online  
%       (http://ccar.colorado.edu/asen5050/projects/projects_2001/benoit/solar_irradiance_on_mars.htm)
%        https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/19890015158.pdf
%        Outputs flux 

% Constants
    time_step = 60;          % Time step in [sec]
    
    sFlux0 = 589.2;          % Solar irradiance at Mars' mean distance from Sun [W/m^2]
    P = 88775;               % Number of seconds in Martian sol [seconds]
    eccentricity = 0.093377; % Mars' planetary orbit eccentricity [no unit]
    LsP = 248;               % Aerocentric longitude at perihelion (when Mars' is closest to Sun) [degrees]... to get 249.5, took average of values forund on http://www-mars.lmd.jussieu.fr/mars/time/solar_longitude.html and the site under assumptions
    E = 24.936;              % Mars obliquity [degrees]
    opticalDepth = 0.5;      % Factor to account for dust levels[no unit] (lower = less dust and higher surface solar flux) 
    
    % Table constants
    opDepthTable = [.1 .2 .3 .4 .5 .6 .7 .8 .9 1 1.1 1.2 1.3 1.4 1.5 1.6 1.7 1.8 1.9 2 2.25 2.50 2.75 3 3.25 3.5 4 5 6];
    zenAngleTable = [0 10 20 30 40 50 60 70 80 85 90];
    valsTable = [.883 .883 .882 .880 .876 .870 .857 .830 .755 .635 0;      % Data Table Source: https://ac.els-cdn.com/0038092X90901567/1-s2.0-0038092X90901567-main.pdf?_tid=e4601e00-1412-11e8-97ae-00000aacb35e&acdnat=1518893389_33db73dec0e65a7bb93a15b7ea47f75a
                 .866 .865 .860 .858 .851 .836 .813 .758 .640 .470 0;
                 .847 .846 .841 .836 .826 .806 .774 .708 .562 .412 0;
                 .828 .827 .821 .815 .802 .778 .740 .667 .502 .373 0;
                 .810 .810 .802 .796 .778 .752 .708 .628 .452 .342 0;
                 .793 .791 .785 .775 .755 .725 .677 .593 .414 .318 0;
                 .776 .773 .766 .755 .733 .700 .646 .555 .383 .298 0;
                 .760 .756 .750 .736 .710 .675 .616 .520 .360 .280 0;
                 .745 .740 .733 .717 .690 .650 .587 .487 .336 .264 0;
                 .732 .725 .717 .700 .670 .628 .560 .455 .317 .252 0;
                 .713 .709 .700 .682 .651 .604 .539 .433 .300 .239 0;
                 .697 .692 .683 .662 .632 .585 .518 .413 .288 .230 0;
                 .682 .677 .667 .646 .613 .567 .498 .394 .273 .220 0;
                 .666 .661 .650 .629 .596 .546 .478 .379 .262 .210 0;
                 .651 .646 .633 .612 .580 .530 .460 .362 .251 .202 0;
                 .637 .630 .618 .597 .563 .512 .441 .348 .240 .195 0;
                 .622 .615 .601 .581 .546 .494 .424 .332 .232 .188 0;
                 .609 .600 .586 .568 .531 .480 .408 .318 .224 .181 0;
                 .596 .587 .571 .551 .514 .464 .393 .304 .217 .176 0;
                 .582 .573 .558 .537 .500 .448 .378 .293 .208 .170 0;
                 .552 .542 .522 .501 .462 .410 .343 .265 .190 .156 0;
                 .518 .509 .492 .469 .430 .378 .316 .242 .174 .145 0;
                 .486 .478 .462 .440 .401 .353 .293 .224 .158 .136 0;
                 .460 .450 .434 .414 .376 .330 .273 .206 .150 .128 0;
                 .434 .424 .410 .390 .354 .308 .254 .193 .140 .120 0;
                 .411 .400 .387 .367 .333 .290 .240 .180 .132 .110 0;
                 .370 .360 .347 .330 .296 .258 .212 .160 .118 .100 0;
                 .294 .286 .275 .258 .230 .203 .166 .130 .094 .080 0;
                 .228 .223 .215 .200 .178 .153 .130 .103 .080 .068 0];
    
    % Calculations
    distanceRatio = (1 + eccentricity * cosd(Ls - LsP)) / (1 - eccentricity * eccentricity); % This distance ratio is confirmed to work, as it outputs a max ratio of 1.103, and 589.2 * 1.103 * 1.103 = 717
    solarDeclination = asind(sind(E) * sind(Ls)); % confirmed to work based on comparing graphs
    
    t_start = -P/(2*pi) * acos(-tand(latitude) * tand(solarDeclination));     % Calculate time of sunrise
    t_end = P/(2*pi) * acos(-tand(latitude) * tand(solarDeclination));        % Calculate time of sunset
    % The entire Martian sol goes from -44387 sec to 44387 sec, where t = 0 sec corresponds to solar noon

    day_length_sec = t_end - t_start;     % Find duration of sunlight

    t = t_start:time_step:t_end;  % Time interval from first second of daylight to last second of daylight, where 0 seconds is noon

    for i = 1:length(t) 

        h = (2 * pi * t(i)) / P;
        cosineZ(i) = (sind(latitude) * sind(solarDeclination)) + (cosd(latitude) * cosd(solarDeclination) * cos(h));
        zenithAngle = acosd(cosineZ(i));
        solarFluxSurfaceAtmosphere = sFlux0 * distanceRatio * distanceRatio; % at top of atmosphere

        func = interp2(zenAngleTable, opDepthTable, valsTable, zenithAngle, opticalDepth); % double interpolating table above to give a coefficient based on optical depth and zenith angle

        Flux(i) = 0; % Preliminarily, flux is set to zero at all times of day, as there is no flux at night
        if (zenithAngle >= 0 && zenithAngle <= 90) % solar radiation only available during the day, which is when zenith angle is 0-90
            Flux(i) = solarFluxSurfaceAtmosphere * func * cosineZ(i) / 0.9; % Equation for total flux on horizontal surface
        end
    end    

    flux_avg = mean(Flux);           % [W/m^2]
    day_length_hr = day_length_sec / 3600; % [hr]

    % Total energy [J/m^2] = flux_avg [W/m^2] * day_length_sec [sec]

end
