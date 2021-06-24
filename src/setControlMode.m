
% setControlMode.m
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


function setControlMode(s, mode)
    
    % Send command to Arduino to change EPROM of all motors to be in
    % either position mode or velocity mode (also known as 'wheel mode').
    
    % External Variables
    % @ s                   - Serial object.
    % @ mode                - A string containing the mode to change 
    %                   the EPROM to. Accepted commands are "pos",
    %                   "position", "vel" & "velocity".
    
    if (mode == "position") || (mode == "pos")
        fprintf(s, '%s', 'cnp\n');
    elseif (mode == "velocity") || (mode == "vel")
        fprintf(s, '%s', 'cnv\n');
    end


end