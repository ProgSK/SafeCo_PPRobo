% Initialize the serial port communication with the Arduino
a = arduino('COM4', 'Uno'); % Initialise Arduino with correct port connection
set(arduino, 'BaudRate', 9600); % Setting Arduino baud rate

fopen(arduino);

try
    while true
        data = fscanf(arduino, '%s');
        
        if strcmp(data, 'STOP')
            disp('MATLAB script is stopped.');
            % Need to add code to stop MATLAB script or pause execution here
            break;
            % pause;
        elseif strcmp(data, 'RESUME')
            disp('MATLAB script is resumed.');
            % Need to add code to resume MATLAB script execution here
            break;
            % pause;
        end
    end
catch
    fclose(arduino);
    delete(arduino);
    clear arduino;
end
