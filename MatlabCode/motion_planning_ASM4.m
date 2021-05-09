%% Robotics System Assignment 4 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

% This will be the motion planning block of the task space velocity control
% architecture.

% This block will take the input as a 2 points on the chessboard and
% construct a list of points/vertices the robot needs to get thru to the
% final pose.

% this will return a struct of each segment

function trajectory = motion_planning_ASM4(Square)

startPoint = Square.startSq;
endPoint = Square.endSq;
[pStart, pEnd] = ChessBoardLocation(startPoint,endPoint);

via_points = [[pStart(1:2);pStart(3)+100],...
                        [pEnd(1:2);pEnd(3)+100]];     % Each column is one via point

[pHome,~] = ChessBoardLocation('home','home');
% interPoint = [pHome(1:2);pHome(3)-100];
interPoint = [400;0;150];

% construct the sets of points for finishing the task
set1.setPoints = [pHome, via_points(:,1), pStart];
set1.distance = getDistance(set1.setPoints);
set2.setPoints = [pStart, via_points(:,1), via_points(:,2), pEnd];
set2.distance = getDistance(set2.setPoints);
set3.setPoints = [pEnd, via_points(:,2), pHome];
set3.distance = getDistance(set3.setPoints);

% define the init and final time of each set
set1.t_i = 0;set1.t_f = 5;
set2.t_i = 0;set2.t_f = 5;
set3.t_i = 0;set3.t_f = 5;

% traj gen from the set points #
[set1.poseRef, set1.velRef, set1.thetaRef, set1.thetaDotRef, set1.omegaOnAxis, set1.R_t_array] = trajectory_generation_ASM4(set1.setPoints, set1.t_i, set1.t_f);
[set2.poseRef, set2.velRef, set2.thetaRef, set2.thetaDotRef, set2.omegaOnAxis, set2.R_t_array] = trajectory_generation_ASM4(set2.setPoints, set2.t_i, set2.t_f);
[set3.poseRef, set3.velRef, set3.thetaRef, set3.thetaDotRef, set3.omegaOnAxis, set3.R_t_array] = trajectory_generation_ASM4(set3.setPoints, set3.t_i, set3.t_f);

% combine them into one struct
trajectory.segment(1) = set1;
trajectory.segment(2) = set2;
trajectory.segment(3) = set3;

    function distance = getDistance(array)
        distance = zeros(1,length(array(1,:))-1);
        for i = 1:length(array(1,:))-1
        
            distance(i) = norm(array(:,i+1)-array(:,i));
        end
    end


end