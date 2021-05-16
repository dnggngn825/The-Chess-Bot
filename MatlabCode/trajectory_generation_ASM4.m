%% Robotics System Assignment 4 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

% This function is for trajectory generation block for task space velocity
% control.
% this function will take a list of points and generate a list of track
% points along the tracjectory

% poseRef will store the pose as each column in the array
% velRef will store the velocity as each column in the array

function [poseRef, velRef, thetaRef, thetaDotRef,omegaOnAxis,R_t_array] = trajectory_generation_ASM4(points, t_i, t_f)

no_segments = length(points(1,:))-1;
time = getTimeArray(points,t_i, t_f);
poseRef = [];velRef = [];
thetaDotRef = [];
thetaRef = [];
omegaOnAxis = [];
R_t_array = [];

% Find segments for trajectories
for i = 1:no_segments
    for j = 1:3
%         [array_coeff(j,:),xAndvWTime(j,:,:)] = solveForSpline(points(j,i),points(j,i+1),0,0,t_i + (i-1)*t_f/no_segments,i*t_f/no_segments);
        [array_coeff(j,:), xAndvWTime(j,:,:)] = solveForSpline(points(j,i),points(j,i+1),0,0,time(i),time(i+1));
        
    end
    poseRef = [poseRef, [xAndvWTime(1,:,1);xAndvWTime(2,:,1);xAndvWTime(3,:,1)]];
    poseRef = round(poseRef,3);
    velRef = [velRef, [xAndvWTime(1,:,2);xAndvWTime(2,:,2);xAndvWTime(3,:,2)]];
    velRef = round(velRef,3);
    [phi_coeff(i,:),phi_omega_time(i,:,:), k_hat(:,i), R] = InterpolatingOrientation(points(:,i),points(:,i+1),time(i),time(i+1));
    thetaDotRef = [thetaDotRef, phi_omega_time(i,:,2)];
    thetaDotRef = round(thetaDotRef,3);
    R_t_array = [R_t_array , R];
    for omega = phi_omega_time(i,:,2)
        omegaOnAxis(:,end+1) = omega*k_hat(:,i) ;
    end
    
    if (size(thetaRef) ~= 0)
        thetaRef = [thetaRef, thetaRef(end)*ones(size(phi_omega_time(i,:,1))) + phi_omega_time(i,:,1)];
    else
        thetaRef = [phi_omega_time(i,:,1)];
    end
    
    
    
end


end