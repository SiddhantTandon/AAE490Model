for i = 1:length(success_blades)
    v_max(i) = success_blades(i).v_max_data(1);
    accel_max(i) = success_blades(i).max_accel_data(1);
    carry_cap(i) = success_blades(i).mass_avail; % *** NEED TO FIX THIS, use Storage struct?
end

max_v_max = max(v_max);
max_accel_max = max(accel_max);
max_carry_cap = max(carry_cap);

nondim_v_max = v_max/max_v_max;
nondim_accel_max = accel_max/max_accel_max;
nondim_carry_cap = carry_cap/max_carry_cap;

for i = 1:length(success_blades)
   success_blades.score(i) = 3*nondim_v_max(i) + 2*nondim_accel_max(i) + 5*nondim_carry_cap(i);
end
