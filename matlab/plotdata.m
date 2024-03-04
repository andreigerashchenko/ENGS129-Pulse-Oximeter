% Read the data from the CSV file
data = csvread('scope_3.csv');

% Extract the time and voltage columns
time = data(:, 1);
voltage1 = data(:, 2);
voltage2 = data(:, 3);

% Shift the time values
time = time - time(1) + 5;

% Plot the data
plot(time, voltage1, 'b', time, voltage2, 'r');

% Add a legend
legend('Infrared', 'Red');

% Add labels and title
xlabel('Time (s)');
ylabel('Voltage');

% Show the plot
grid on;
