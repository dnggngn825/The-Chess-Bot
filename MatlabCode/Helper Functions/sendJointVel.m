
% sendJointVel.m
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


function sendJointVel(s, jointVel, numMotors)
    
    % Send joint velocities to Arduino over serial. This is done by
    % first converting the floats to a string and then adding a
    % terminating non-numeric string character.
    
    % External Variables
    % @ s                   - Serial object.
    % @ jointVel            - Vector of joint velocites to be sent
    %                   to each motor in rad/s.
    
    
    % Send drive motor command
    fprintf(s, '%s', 'vel\n');
    fprintf(s, '%s', 'vel\n');

    % Send joint velocities individually
    
    for i=1:numMotors
        fprintf(s, '%s', num2str(jointVel(i), '%.5f') + "e");
    end
    
    % wait for arduino ack
    while ( fread(s, 1, 'uchar') ~= 'd' ) 
    end

end
