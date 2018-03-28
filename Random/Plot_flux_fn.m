
% This script makes a contour plot of the outputs of the solar flux
% function to help understand when/where the most desirable and least
% desirable configurations occur (Takes long time to run)

clear;close all;clc



mission_lat = -90:5:90;      %range of vaules to be tested [deg]
solar_lon = 0:5:360;

year_days = linspace(1,668.6,length(solar_lon));      % Convert units of X axis to days instead of degrees 

for i = 1:length(mission_lat)
    for j = 1:length(solar_lon)

        [ solar_flux(i,j), sun_time(i,j) ] = solarFlux(mission_lat(i), solar_lon(j));     % Average solar flux on Martian surface (assumed constant) [W/m^2] and % Hours of useful sunlight on Martian surface [hr]
        
        if imag( sun_time(i,j)) ~= 0    % Remove complex solutions 
            sun_time(i,j) = NaN;
            solar_flux(i,j) = NaN;
        end
 
    end
end

figure(1)
contourf(year_days, mission_lat,solar_flux,100,'linecolor','none')
colorbar
title('Average Solar Flux [W/m^2] Contour Plot')
ylabel('Surface Latitude [deg N]')
xlabel('Martian Sols After Vernal Equinox [Sols]')

figure(2)
contourf(year_days, mission_lat,sun_time,100,'linecolor','none')
colorbar
title('Duration of Daylight [hr] Contour Plot')
ylabel('Surface Latitude[deg N]')
xlabel('Martian Sols After Vernal Equinox [Sols]')

figure(3)
energy_avail = solar_flux .* sun_time .* 3600;   % [J/m^2]
contourf(year_days, mission_lat,energy_avail,100,'linecolor','none')
colorbar
title('Daily Energy Density [J/m^2] Contour Plot')
ylabel('Surface Latitude [deg N]')
xlabel('Martian Sols After Vernal Equinox [Sols]')

% Add ROI to plot
hold on
plot([0,668.6],[18.86, 18.86],'k-')
annotation('textarrow',[0.45 0.48],[0.67 0.61],'String','Region of Interest','Fontsize',15)

