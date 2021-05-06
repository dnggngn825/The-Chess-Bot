%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

clear all;
clc;
%% Define variables


%% 2.1: Forward Kinematics (FK)
syms d1 d2 d3 d4 Q1 Q2 Q3 Q4 real

% Calculate the transformation matrices from DH table
T_1to0 = TransformationMatrix_1to0(0,0,d1,Q1);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,Q2);
T_3to2 = TransformationMatrix_1to0(0,d2,0,Q3);
T_4to3 = TransformationMatrix_1to0(0,d3,0,Q4+pi/2);
T_Eto4 = TransformationMatrix_1to0(pi/2,0,d4,0);

% Final transformation matrix expressing the end-effector pose with respect
% to the intertial frame
T_4to0 = T_1to0 * T_2to1 * T_3to2 * T_4to3;
T_4to0 = simplify(T_4to0);

% Symbolic form of the transformation matrix 
fprintf("The final transformation matrix expressing the end-effector pose with respect to the inertial frame\n");
T_Eto0 = simplify(T_4to0*T_Eto4)

%% Produce the end-effector pose given trivial joint displacement using Forward Kinematics

% q = [pi/10; pi/3; -pi*3/4; 0];
q= [ 0 0 0 0]';

endEffectorPoseInframe4 = [0 -d4 0 1]';
endEffectorPoseInframeE = [0 0 0 1]';

T_4to0 = simplify(subs(T_4to0,{Q1,Q2,Q3,Q4},{q(1),q(2),q(3),q(4)}));

endEffectorPoseInframe0 = T_4to0 * endEffectorPoseInframe4;

% End effector pose for q = [0;0;0;0]
endEffectorPoseInframe0 = vpa(subs(endEffectorPoseInframe0, {d1,d2,d3,d4},{200,200,200,80}))

% FK Verification
stickplot();


%% Determine the Link Length

% Link length is adjusted in "RobotSpec.m" for visualization different
% working ranges of both Joint 4 and End-effector.

WorkRangeVisualization(); % Change variable N in line 13 for better visualization

%% 2.4.2: Inverse Kinematics

% Verify Inverse Kinematics
IKVerification();



