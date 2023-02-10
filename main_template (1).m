%% ENGD1025 Assignment Template
%initialisation section; do not remove this section 
clear all; %clear all variables
clc %clear command window
close all % close all open figures

%% Put your name, surname and student ID below
name='Daniel'
surname='Paluszczyszyn'
sid='P123456789'; %your student ID number;
 
%% Your script should start below:
% Question 2.1
% Define the file name
filename = 'Log_U0006387_160311_740d0.csv';

% Read the file into a table data structure
T = readtable(filename);

%   Question 2.2 (1)
% Extract the required variables from the table
Time = T.Time; % Time in seconds
Speed = T.Speed.*(18/5); % Speed in km/h
Gear = T.Gear; % Gear
Elv = T.Elv; % Elevation in meters
SOC = T.SOC; % SOC in %
PackVolts = T.PackVolts; % Battery voltage in V
PackAmps = T.PackAmps; % Battery current in A

% Question 2.2 (2)

% Create subplots
figure;
subplot(3, 2, 1);
plot(Time, Speed, 'Color', 'r');
xlabel('Time (s)');
ylabel('Speed (km/h)');

subplot(3, 2, 2);
plot(Time, Gear, 'Color', 'g');
xlabel('Time (s)');
ylabel('Gear');

subplot(3, 2, 3);
plot(Time, Elv, 'Color', 'b');
xlabel('Time (s)');
ylabel('Elevation (m)');

subplot(3, 2, 4);
plot(Time, SOC, 'Color', 'c');
xlabel('Time (s)');
ylabel('SOC (%)');

subplot(3, 2, 5);
plot(Time, PackVolts, 'Color', 'm');
xlabel('Time (s)');
ylabel('Pack Volts (V)');

subplot(3, 2, 6);
plot(Time, PackAmps, 'Color', 'y');
xlabel('Time (s)');
ylabel('Pack Amps (A)');

% Question 2.3 (1)
% Extract the data columns
MotorPwr = T.MotorPwr; % Motor power in 50W
AuxPwr = T.AuxPwr; % Aux power in 50W
% A_CPwr = T.A_CPwr100w; % A/C power in 100W

% Plot the data
figure;
subplot(3,1,1);
plot(MotorPwr, 'r');
xlabel('Time (s)');
ylabel('Motor Power (50W)');

subplot(3,1,2);
plot(AuxPwr, 'g');
xlabel('Time (s)');
ylabel('Aux Power (50W)');

subplot(3,1,3);
% plot(A_CPwr, 'b');
xlabel('Time (s)');
ylabel('A_C Power (100W)');



% Question 2.3 (2)
% Convert to kW
MotorPwr = T.MotorPwr * 0.05; % 1 50W = 0.05kW
AuxPwr = T.AuxPwr * 0.05; % 1 50W = 0.05kW
% A_CPwr = A_CPwr * 0.1; % 1 100W = 0.1kW

% Question 2.3 (3)

% Calculate battery power in W
% Extract battery voltage and current data
BatteryVoltage = T.PackVolts; % Battery voltage in V
BatteryCurrent = T.PackAmps; % Battery current in A

% Question 2.3 (4)
% Calculate battery power in W
BatteryPower = BatteryVoltage .* BatteryCurrent;

% Calculate the battery power in W
P = T.PackVolts .* T.PackAmps;

% Convert the powers from W to kW
MotorPwr_kW = MotorPwr / 1000;
AuxPwr_kW = AuxPwr / 1000;
%A_CPwr_kW = A_CPwr / 1000;
P_kW = P / 1000;

% Plot the powers against time
figure;
plot(T.Time, MotorPwr_kW, '-', 'LineWidth', 1.5);
hold on;
plot(T.Time, AuxPwr_kW, '--', 'LineWidth', 1.5);
%plot(T.Time, A_CPwr_kW, ':', 'LineWidth', 1.5);
plot(T.Time, P_kW, '-.', 'LineWidth', 1.5);

% Add labels to the x and y axes
xlabel('Time (s)');
ylabel('Power (kW)');

% Add a title to the figure
title('Powers vs Time');

% Add a legend to the figure
legend({'Motor Power', 'Aux Power', 'A/C Power', 'Battery Power'});

% Add grid to the figure
grid on;

%Question 2.4 (1)

% Define variables to store acceleration values
acceleration = zeros(length(Time)-1, 1);

% Calculate acceleration using the formula
for i = 1:length(Time)-1
acceleration(i) = (Speed(i+1) - Speed(i)) / (Time(i+1) - Time(i)) * 3.6 / 1000; %converting km/h to m/s
end

% Store the acceleration values in a new variable
Acceleration = acceleration;

% Question 2.4(2)
% Define the ranges for each driving style
economic = [0.7, 2.3];
normal = [2.31, 3.3];
aggressive = [3.31, 8.5];

% Get the size of the acceleration values
n = length(acceleration);

% Initialize the ds_mapping variable to store the values
ds_mapping = zeros(1, n);

% Loop through each acceleration value
for i = 1:n
% Get the current acceleration value
acc = acceleration(i);

% Check if the acceleration value falls within the economic range
if acc >= economic(1) && acc <= economic(2)
ds_mapping(i) = 1;
% Check if the acceleration value falls within the normal range
elseif acc >= normal(1) && acc <= normal(2)
ds_mapping(i) = 2;
% Check if the acceleration value falls within the aggressive range
elseif acc >= aggressive(1) && acc <= aggressive(2)
ds_mapping(i) = 3;
% If the acceleration value falls outside the defined ranges, assign 0
else
ds_mapping(i) = 0;
end
end

%Question 2.4 (3)

% Sample data
Time = [0, 1, 2, 3, 4, 5];
speed = [0, 10, 20, 30, 40, 50];

% Calculate acceleration
delta_t = Time(2:end) - Time(1:end-1);
delta_v = speed(2:end) - speed(1:end-1);
acceleration = delta_v./delta_t;

% Mapping of driving style
ds_mapping = zeros(size(acceleration)); % initialize ds mapping to store values
for i = 1:length(acceleration)
    if acceleration(i) >= 0.7 && acceleration(i) <= 2.3
        ds_mapping(i) = 1;
    elseif acceleration(i) > 2.3 && acceleration(i) <= 3.3
        ds_mapping(i) = 2;
    elseif acceleration(i) > 3.3 && acceleration(i) <= 8.5
        ds_mapping(i) = 3;
    end
end

% Plotting
figure
subplot(2,1,1)
plot(Time(2:end), acceleration, 'LineWidth', 2)
title('Acceleration vs Time')
xlabel('Time (s)')
ylabel('Acceleration (m/s^2)')

subplot(2,1,2)
plot(Time(2:end), ds_mapping, 'LineWidth', 2)
title('Driving Style Mapping vs Time')
xlabel('Time (s)')
ylabel('Driving Style (1-Economic, 2-Normal, 3-Aggressive)')

%Question 2.5 (1)

% Read the data from the CSV file
filename = 'Log_U0006387_160311_740d0.csv'; % specify the name of the file
T = readtable(filename); % read the data into a table

% Extract the columns of interest
PackT1C = T.PackT1C; % extract the Pack T1 C column
PackT2C = T.PackT2C; % extract the Pack T2 C column
PackT3C = T.PackT3C; % extract the Pack T3 C column
PackT4C = T.PackT4C; % extract the Pack T4 C column


%Question 2.5 (2)


% Calculate the minimum, maximum, and average temperature for each sensor
min_t1 = min(PackT1C);
max_t1 = max(PackT1C);
avg_t1 = mean(PackT1C);
min_t2 = min(PackT2C);
max_t2 = max(PackT2C);
avg_t2 = mean(PackT2C);
min_t3 = min(PackT3C);
max_t3 = max(PackT3C);
avg_t3 = mean(PackT3C);
min_t4 = min(PackT4C);
max_t4 = max(PackT4C);
avg_t4 = mean(PackT4C);

% Display the results
fprintf('Sensor 1: Min: %.2f, Max: %.2f, Average: %.2f\n', min_t1, max_t1, avg_t1);
fprintf('Sensor 2: Min: %.2f, Max: %.2f, Average: %.2f\n', min_t2, max_t2, avg_t2);
fprintf('Sensor 3: Min: %.2f, Max: %.2f, Average: %.2f\n', min_t3, max_t3, avg_t3);
fprintf('Sensor 4: Min: %.2f, Max: %.2f, Average: %.2f\n', min_t4, max_t4, avg_t4);


%Question 2.5 (3)

% Calculate minimum, maximum and average temperature for each sensor
min_temp = min(filename);
max_temp = max(filename);
avg_temp = mean(filename);

% Plot the temperature values for each sensor
bar_data = [min_temp; max_temp; avg_temp];
bar(bar_data);

% Add legend, grid, title, and labels to the plot
legend({'Sensor 1', 'Sensor 2', 'Sensor 3', 'Sensor 4'});
grid on;
title('Temperature Statistics for each Sensor');
xlabel('Statistics');
ylabel('Temperature (°C)');
set(gca, 'XTickLabel', {'Min', 'Max', 'Avg'});


