load('analyzed_blades.mat');

for i = 1:length(analyzed_blades)
    blade_data = analyzed_blades(i);
    v_max(i) = blade_data.v_max_data(1);
    accel_max(i) = blade_data.max_accel_data(1);
    carry_cap(i) = blade_data.mass_avail;
end

max_v_max = max(v_max);
max_accel_max = max(accel_max);
max_carry_cap = max(carry_cap);

nondim_v_max = v_max/max_v_max;
nondim_accel_max = accel_max/max_accel_max;
nondim_carry_cap = carry_cap/max_carry_cap;

max_score = 0;
sec_score = 0;
thr_score = 0;
max_score_index = 0;
sec_score_index = 0;
thr_score_index = 0;

for i = 1:length(analyzed_blades)
    score = 3*nondim_v_max(i) + 2*nondim_accel_max(i) + 5*nondim_carry_cap(i);
    analyzed_blades(i).score = 3*nondim_v_max(i) + 2*nondim_accel_max(i) + 5*nondim_carry_cap(i);
    if score > max_score
        thr_score = sec_score;
        thr_score_index = sec_score_index;
        sec_score = max_score;
        sec_score_index = max_score_index;
        max_score = score;
        max_score_index = i;
    end
end

fprintf('First place:\n');
disp(analyzed_blades(max_score_index));
fprintf('Second place:\n');
disp(analyzed_blades(sec_score_index));
fprintf('Third place:\n');
disp(analyzed_blades(thr_score_index));
