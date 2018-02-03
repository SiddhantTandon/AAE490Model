clc
clear
load('data_store.mat');

n_drone_list = [];
fly_t_list = [];
h_cruise_list = [];
propVer_list = [];
propHor_list = [];
prop_n_ver_list = [];
prop_n_hor_list = [];
prop_r_ver_list = [];
prop_r_hor_list = [];
wing_percent_lift_list = [];
score_list = [];

for index = 1:length(data_store)
    n_drone_list(index) = data_store(index).n_drone;
    fly_t_list(index) = data_store(index).fly_t;
    h_cruise_list(index) = data_store(index).h_cruise;
    propVer_list(index) = data_store(index).propVer;
    propHor_list(index) = data_store(index).propHor;
    prop_n_ver_list(index) = data_store(index).prop_n_ver;
    prop_n_hor_list(index) = data_store(index).prop_n_hor;
    prop_r_ver_list(index) = data_store(index).prop_r_ver;
    prop_r_hor_list(index) = data_store(index).prop_r_hor;
    wing_percent_lift_list(index) = data_store(index).wing_percent_lift;
    score_list(index) = data_store(index).score;
end

n_drone_data = n_drone_list(1);
fly_t_data = fly_t_list(1);
h_cruise_data = h_cruise_list(1);
prop_n_ver_data = prop_n_ver_list(1);
prop_r_ver_data = prop_r_ver_list(1);

for index = 1:length(n_drone_list)
    if ~ismember(n_drone_list(index), n_drone_data)
         n_drone_data = [n_drone_data, n_drone_list(index)];
    end
    if ~ismember(fly_t_list(index), fly_t_data)
         fly_t_data = [fly_t_data, fly_t_list(index)];
    end
    if ~ismember(h_cruise_list(index), h_cruise_data)
         h_cruise_data = [h_cruise_data, h_cruise_list(index)];
    end
    if ~ismember(prop_n_ver_list(index), prop_n_ver_data)
         prop_n_ver_data = [prop_n_ver_data, prop_n_ver_list(index)];
    end
    if ~ismember(prop_r_ver_list(index), prop_r_ver_data)
         prop_r_ver_data = [prop_r_ver_data, prop_r_ver_list(index)];
    end
end

n_drone_data = sort(n_drone_data);
n_drone_success = zeros(1, length(n_drone_data));
fly_t_data = sort(fly_t_data);
fly_t_success = zeros(1, length(fly_t_data));
h_cruise_data = sort(h_cruise_data);
h_cruise_success = zeros(1, length(h_cruise_data));
prop_n_ver_data = sort(prop_n_ver_data);
prop_n_ver_success = zeros(1, length(prop_n_ver_data));
prop_r_ver_data = sort(prop_r_ver_data);
prop_r_ver_success = zeros(1, length(prop_r_ver_data));

for index = 1:length(n_drone_data)
    for index2 = 1:length(n_drone_list)
        if n_drone_list(index2) == n_drone_data(index)
            n_drone_success(index) = n_drone_success(index) + 1;
        end
    end
end

for index = 1:length(fly_t_data)
    for index2 = 1:length(fly_t_list)
        if fly_t_list(index2) == fly_t_data(index)
            fly_t_success(index) = fly_t_success(index) + 1;
        end
    end
end

for index = 1:length(h_cruise_data)
    for index2 = 1:length(h_cruise_list)
        if h_cruise_list(index2) == h_cruise_data(index)
            h_cruise_success(index) = h_cruise_success(index) + 1;
        end
    end
end

for index = 1:length(prop_n_ver_data)
    for index2 = 1:length(prop_n_ver_list)
        if prop_n_ver_list(index2) == prop_n_ver_data(index)
            prop_n_ver_success(index) = prop_n_ver_success(index) + 1;
        end
    end
end

for index = 1:length(prop_r_ver_data)
    for index2 = 1:length(prop_r_ver_list)
        if prop_r_ver_list(index2) == prop_r_ver_data(index)
            prop_r_ver_success(index) = prop_r_ver_success(index) + 1;
        end
    end
end

%% Output
figure(1);
subplot(2,5,1);
plot(n_drone_list, score_list, 'o');
xlabel('Number of drones');
ylabel('Score');
subplot(2,5,2);
plot(fly_t_list, score_list, 'o');
xlabel('Flight time (s)');
ylabel('Score');
subplot(2,5,3);
plot(h_cruise_list, score_list, 'o');
xlabel('Cruising altitude (m)');
ylabel('Score');
subplot(2,5,4);
plot(prop_n_ver_list, score_list, 'o');
xlabel('Number of vertical motors');
ylabel('Score');
axis([1,5,6,16]);
subplot(2,5,5);
plot(prop_r_ver_list, score_list, 'o');
xlabel('Radius of vertical blades (m)');
ylabel('Score');

subplot(2,5,6);
plot(propVer_list, score_list, 'o');
xlabel('Vertical propulsion type');
ylabel('Score');
subplot(2,5,7);
plot(propHor_list, score_list, 'o');
xlabel('Horizontal propulsion type');
ylabel('Score');
subplot(2,5,8);
plot(prop_n_hor_list, score_list, 'o');
xlabel('Number of horizontal motors');
ylabel('Score');
subplot(2,5,9);
plot(prop_r_hor_list, score_list, 'o');
xlabel('Radius of horizontal blades (m)');
ylabel('Score');
subplot(2,5,10);
plot(wing_percent_lift_list, score_list, 'o');
xlabel('Percentage of wing contribution to lift');
ylabel('Score')

figure(2);
subplot(2,5,1);
plot(n_drone_list, score_list, 'o');
xlabel('Number of drones');
ylabel('Score');
subplot(2,5,2);
plot(fly_t_list, score_list, 'o');
xlabel('Flight time (s)');
ylabel('Score');
subplot(2,5,3);
plot(h_cruise_list, score_list, 'o');
xlabel('Cruising altitude (m)');
ylabel('Score');
subplot(2,5,4);
plot(prop_n_ver_list, score_list, 'o');
xlabel('Number of vertical motors');
ylabel('Score');
subplot(2,5,5);
plot(prop_r_ver_list, score_list, 'o');
xlabel('Radius of vertical blades (m)');
ylabel('Score');

subplot(2,5,6);
bar(n_drone_data,n_drone_success);
xlabel('Number of drones');
ylabel('Successes');
subplot(2,5,7);
bar(fly_t_data,fly_t_success);
xlabel('Flight time (s)');
ylabel('Successes');
axis([200,1000,0,180]);
xticks([200,400,600,800,1000]);
subplot(2,5,8);
bar(h_cruise_data,h_cruise_success);
xlabel('Cruising altitude (m)');
ylabel('Successes');
axis([100,400,0,200]);
subplot(2,5,9);
bar(prop_n_ver_data,prop_n_ver_success);
xlabel('Number of vertical motors');
ylabel('Successes');
subplot(2,5,10);
bar(prop_r_ver_data,prop_r_ver_success);
xlabel('Radius of vertical blades (m)');
ylabel('Successes');