
% establishSerial.m
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




function [s] = establishSerial(port, baudrate)

    % Create a serial object and handshake with Arduino
    
    % External Variables
    % @ port                - COM port address of the Arduino.
    % @ baudrate            - BaudRate used between MATLAB and Arduino.
    
    % Create serial object
    s = serial(port,'BaudRate', baudrate); 
    fopen(s);

    % Wait for arduino to send connection request
    while ( fread(s, 1, 'uchar') ~= 'a' ) 
    end

    % Send back ack to arduino
    fprintf(s, '%c', 'a'); 

    % Wait for arduino ack
    while ( fread(s, 1, 'uchar') ~= 'b' ) 
    end

    % Consume a NaN
    temp = fscanf(s,'%s');

end