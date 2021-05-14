
% readFB.m
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


function [fb, e] = readFB(s, numMotors)

    % Read motor position by sending a position read command to the
    % Arduino and then calling the readSerial() function to receive
    % the data.
    
    % External Variables
    % @ s                   - Serial object.
    % @ numMotors           - Number of motors expected to read from.
    
    % Internal Functions
    % @ readSerial()        - Read data sent by Arduino over serial.
    
    % Send position read command to Arduino
    fprintf(s, '%s', 'fbk\n');
    fprintf(s, '%s', 'fbk\n'); 

    % Convert Motor Encoder Data To Radians
    SCS15_2_RAD = 0.00366450;
    SCS009_2_RAD = 0.00511327;
    
    % Read motor feedback in SCS format
    [fb, e] = readSerial(s, numMotors);
    
    % Convert from SCS to rad
    for i = 1:numMotors
        if (i == 5 || i == 6)
            fb(i) = fb(i)*SCS009_2_RAD;
        else
            fb(i) = fb(i)*SCS15_2_RAD;
        end
    end

end