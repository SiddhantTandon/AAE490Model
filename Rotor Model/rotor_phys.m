clc;clear;
load('ana_high_fid_blade.mat')
thick = 0.02;
rho_blade = 1200;

for blade = 1:length(analyzed_blades)
    blade_data = analyzed_blades(blade);
    radius = blade_data.radius;
    r_min = blade_data.min_radius;
    num_blade = blade_data.blade_number;
    sections = length(blade_data.chords);
    r_pos = 0.1*radius + ((1:sections) - .5)*(radius-r_min)/sections;
    masses = thick * (blade_data.chords .^2) * (r_pos(2) - r_pos(1)) * rho_blade;
    mass = sum(masses);
    inertia = sum(num_blade * masses .* (r_pos .^2));
    
    % Store data
    analyzed_blades(blade).mass = mass;
    analyzed_blades(blade).inertia = inertia;
end