function [polyCoefs, maxRe] = BestFit_cl_v_Re(~)
    % Adjustable metrics
    degree = 21; % degree polynomial desired
    maxRe = 400000; % maximum Reynolds number to be modeled

    % Load data from XFLR
    data = load('cl_v_Re_data_revised.txt');
    Re_data = data( : , 2); % actual Re data
    cl_data = data( : , 3); % actual cl data

    % adds effectively constant cl values for Re > 150,000
    Re_data = [0; Re_data; linspace(151000, maxRe, 100)'];
    cl_data = [0; cl_data; 1.836 * ones(100, 1)];

    % Create Polynomail best fit
    Re = linspace(0, maxRe * 1.1, 10000); % Reynolds number range to plot
    polyCoefs = polyfit(Re_data, cl_data, degree);

    cl = zeros(1, length(Re));
    for n = 1 : degree + 1
        power = degree - n + 1;
        cl = cl + polyCoefs(n) .* (Re .^ power);
    end

    %{
    % Plot Results
    figure(4)
    plot(Re, cl);
    hold on
    plot(Re_data, cl_data, '.');
    plot([42000, 42000], [0, 2])
    plot([54000, 54000], [0, 2])
    plot([80000, 80000], [0, 2])
    plot([maxRe, maxRe], [0, 2])
    axis([0, maxRe * 1.1, 0, 2])
    hold off
    
    fileID = fopen('f_of_Re_coefficients.txt', 'w');
    for n = 1 : length(polyCoefs)
        fprintf(fileID, '%e\n', polyCoefs(n));
    end
    fclose(fileID);
    %}
end