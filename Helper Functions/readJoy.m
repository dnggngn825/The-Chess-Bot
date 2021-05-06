
% readJoy.m
% --------------------------
% Licenting Information: You are free to use or extend this project
% for educational purposes provided that (1) you do not distribute or
% publish solutions, (2) you retain this notice, and (3) you provide 
% clear attribution to the University of Melbourne, Department of 
% Mechanical Engineering.
% 
% Attribution Information: The ChessBot project was developed at the
% University of Melbourne. The core project was primarily developed
% by Professor Denny Oetomo (doetomo@unimelb.edu.au). The ChessBot 
% Skeleton Code was developed by Nathan Batham 
% (nathan.batham@unimelb.edu.au)


function [xJoy, yJoy, e] = readJoy(s)
    
    % Read joystick input by sending a joy read command to the
    % Arduino and then calling the readSerial() function to receive
    % the data.
    
    % External Variables
    % @ s                   - Serial object.
    
    % Internal Variables
    % @ axis                - Number of axis of data to read (e.g. x & y)
    
    % Internal Functions
    % @ readSerial()        - Read data sent by Arduino over serial.


    axis = 2;
    
    % Send joystick read command to Arduino
    fprintf(s, '%s', 'joy\n');

    % Read joystick positions
    [joyData, e] = readSerial(s, axis);
    
    % Assign read axis data to variables
    xJoy = joyData(1);
    yJoy = joyData(2);
    
    
    % Calibrate joystick data
    % ------- STUDENT CODE --------
    
end
    