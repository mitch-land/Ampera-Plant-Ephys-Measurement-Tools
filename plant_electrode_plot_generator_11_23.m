% 1. Configure import options so MATLAB parses the ISO 8601 timestamp natively

% C:\Users\mitch\OneDrive\Documents\MATLAB\Ampera\plant_2026-11-20_16-01-56EST_000.csv
opts = detectImportOptions('C:\Users\mitch\OneDrive\Documents\MATLAB\Ampera\plant_2026-11-20_16-01-56EST_000.csv');
opts.DataLines = [5, Inf]; % Force data reading to start at row 2 through the end of the file

opts = setvartype(opts, {'record_type', 'stim_timestamp_est'}, 'string');      % Column 1: Sample Type Identifier
opts = setvartype(opts, {'timestamp_utc'}, 'datetime');    % Column 2: Timestamp
opts = setvaropts(opts, 2, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss''Z''', 'TimeZone', 'UTC'); %utc or est???
opts = setvartype(opts, 3:9, 'double');    % time_s (time since start) and 5 plant samples = 6 floats

T = readtable('C:\Users\mitch\OneDrive\Documents\MATLAB\Ampera\plant_2026-11-20_16-01-56EST_000.csv', opts);

targetWord = "electrode";
id_electrode = startsWith(T.(1), targetWord); % makes array of first column, makes idx a T/F column matrix of rows starting w electrode
T_filtered = T(id_electrode, :) % keep only rows where idx is true and all columns (selects electrode rows)

TT = table2timetable(T_filtered(:, 2:8), 'RowTimes', T_filtered.(2)); %makes timetable using time_s as independent

stackedplot(TT);
%one variable plot
%plot(TT.Time, TT.(1)); % Plots the 1st float column against time 
