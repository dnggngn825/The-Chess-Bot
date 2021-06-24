% setPID.m
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


function changeID(s, oldID, newID)
    
    if s.BytesAvailable > 0
        fread(s, s.BytesAvailable)
    end

    % Send change ID command
    fprintf(s, '%s', 'cid\n');
    
    fprintf(s, '%s', num2str(oldID) + "e");
    fprintf(s, '%s', num2str(newID) + "e");
    
    
    % wait for arduino ack
    while ( fread(s, 1, 'uchar') ~= 'd' ) 
    end

end