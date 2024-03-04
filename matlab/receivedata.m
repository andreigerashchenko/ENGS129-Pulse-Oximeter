% Close the serial port
if exist('s', 'var')
    clear s;
end

% Define the serial port parameters
baudrate = 9600;  % Replace with the appropriate baud rate

% Find available serial ports
ports = serialportlist;

% Select the first available serial port
if ~isempty(ports)
    port = ports{1};
    disp(['Using serial port: ', port]);
else
    error('No available serial ports found.');
end

% Open the serial port
s = serialport(port, baudrate);

% Initialize variables for storing received data
data1 = [];
time1 = [];
data2 = [];
time2 = [];

% Create a figure for the plot
figure;

% Set up the plot with separate lines for each channel
hold on;
plot1 = plot(0, 0, 'b'); % Channel 1 plot
plot2 = plot(0, 0, 'r'); % Channel 2 plot
hold off;

% Set plot properties
xlabel('Time (seconds)');
ylabel('Voltage');
title('ADC Data');
legend('Channel 1', 'Channel 2');

% Wait for the microcontroller to send READY
while true
    % Read the incoming data from the serial port
    incoming_data = readline(s);
    
    % Check if the microcontroller sent READY
    if strcmp(incoming_data, 'READY')
        disp('Microcontroller is ready.');
        break;
    end
end

% Send START signal to the microcontroller
writeline(s, 'START');
disp('START signal sent to the microcontroller.');

% Receive and plot data continuously

% typedef struct {
%     uint32_t timestamp; // Timestamp in milliseconds
%     uint8_t data[2]; // 12-bit ADC data, leading 4 bits are 0
%     uint8_t channel; // 1 for channel 1, 2 for channel 2
%     char end = '\n'; // Newline character to delimit data
% } adc_data_t;

while true
    % Read data up to the newline character, decode and print
    incoming_data = readline(s);
    
    % Check if incoming_data has enough elements
    if length(incoming_data) >= 7
        % Extract the timestamp from first 4 bytes
        timestamp = typecast(uint8(incoming_data(1:4)), 'uint32');
        % Extract the data from the next 2 bytes
        data = typecast(uint8(incoming_data(5:6)), 'uint16');
        % Extract the channel from the next byte
        channel = typecast(uint8(incoming_data(7)), 'uint8');
        % Print the received data
        disp(['Timestamp: ', num2str(timestamp), ', Data: ', num2str(data), ', Channel: ', num2str(channel)]);
    else
        % Print size of incoming_data and raw data
        disp(['Size of incoming_data: ', num2str(length(incoming_data)), ', Raw data: ', incoming_data]);

    end
end
