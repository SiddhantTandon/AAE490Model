
% Generate simple line plot of  solar energy density vs time
% of year for the partucular latitude at the region of interest. Solar
% energy density is interpolated to the correct latitude. 

clear;close all;clc

load('Results_of_Plot_flux_fn.mat')

ROI_lat = 18.86;  % [deg]

for i = 1:length(year_days)
    
    enerrgy_ROI(i) = interp1(mission_lat,energy_avail(:,i),ROI_lat);   % linearly interpolate the energy density to the specific latitude of interest
   
end

[min_energy_dens, index] = min(enerrgy_ROI);

fprintf('Solar longitude where minumum energy density occurs: %.0f\n\n',solar_lon(index))

hold on
plot(year_days,enerrgy_ROI)
plot(year_days,min_energy_dens*ones(length(year_days)),'r--')
axis([-inf inf 0 max(enerrgy_ROI)+2e6])
title('Daily Energy Density vs Time of Year at the ROI')
ylabel('Daily Energy Density [J/m^2]')
xlabel('Martian Sols After Vernal Equinox [Sols]')
legend('Energy Density vs. Time','Minumum Energy Density Across Year','location','south')



