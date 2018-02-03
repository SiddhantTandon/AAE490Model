
%The purpose of this code is to determine the best design iteration. To do
%this, it takes in all of the data provided by the analysis code, finds the
%minimum or maximum values for the desired variables (minimize mass,
%maximize flight time, etc), weights all of the values treating the
%minimum/maximum as ideal, and then adds the values for each design before
%outputting the best design at the end.

function data = func_weight(data)

    % Collect inputs
    mass = [];
    SA = [];
    CVel = [];
    Ctime = [];
    PropR = [];
    SysMass = [];
    
    system_count = length(data);
    for index = 1:system_count;
        mass = [mass, data(index).m_sys];
        SA = [SA, data(index).area_pan];
        CVel = [CVel, data(index).vel_min];
        Ctime = [Ctime, data(index).cruise_t];
        PropR = [PropR, data(index).prop_r_ver];
        SysMass = [SysMass, (data(index).m_sys .* data(index).n_drone)];
    end
    
    %Find the minimum values for every weighted decision
    %These are not the final values but considered the best in all outputs
    min_Mass = min(mass);       %Minimum single drone mass in kg
    min_SA = min(SA);           %Minimum solar panel area in m
    max_CVel = max(CVel);       %Maximum cruise velocity in m/s
    max_Ctime = max(Ctime);     %Maximum cruise time s
    min_PropR = min(PropR);     %Minimum prop radius in m
    min_SysMass = min(SysMass); %Mimimum system mass in kg

    %Determine the number of iterations and initial best values
    list_len = size(mass, 2);

    %Determine weightings for each system
    for i = 1:list_len
        Mass_score(i) = 5 * min_Mass/mass(i);
        SA_score(i) = 4 * min_SA/SA(i);
        CVel_score(i) = 4 * CVel(i)/max_CVel;
        Ctime_score(i) = 5 * Ctime(i)/max_Ctime;
        PropR_score(i) = 2 * min_PropR/PropR(i);
        SysMass_score(i) = 1 * min_SysMass/SysMass(i);

        data(i).score = Mass_score(i) + SA_score(i) + CVel_score(i) + Ctime_score(i) + PropR_score(i) + SysMass_score(i);
    end
end


