%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen


function [d1,d2,d3,d4,motorWidth, bigMotorRange, smallMotorRange, minAngle] = RobotSpec()
    
    bigMotorRange = 215/360*2*pi;
    smallMotorRange = 300/360*2*pi;
    minAngle = 5*pi/180;
    
    % First attempt - too short
    d1 = 30;
    d2 = 30; % length of Link 1 in mm
    d3 = 30; % length of Link 2 in mm
    d4 = 30;
    
    
    % Second attempt - work
    d1 = 100;
    d2 = 300-68; % length of Link 1 in mm
    d3 = 300-68; % length of Link 2 in mm
    d4 = 80;

    motorWidth = 68; % width of motor, excluding bracket

    d2 = d2 + motorWidth;
    d3 = d3 + motorWidth;
    d1 = d1/1000;d2 = d2/1000;d3 = d3/1000;d4 = d4/1000;
end