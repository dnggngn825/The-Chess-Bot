%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

%% Description

% This function take the input as the coordinate of the end effector in
% REFERENCE frame and calculate the required Q1, Q2, Q3, Q4 for the arm to
% reach to that point.

% 
function VisualizationJointSpace(Q)
    
    [d1,d2,d3,d4,motorWidth, bigMotorRange, smallMotorRange, minAngle] = RobotSpec();
    
    
    % Link in frame 2 and 3
    Q1 = Q(1);
    Q2 = Q(2);
    Q3 = Q(3);
    Q4 = Q(4);
    
    Link_4 = [0;-d4;0;1];
    Link_3 = [d3;0;0;1];
    Link_2 = [d2;0;0;1];
    
    T_1to0 = TransformationMatrix_1to0(0,0,d1,Q1);
    T_2to1 = TransformationMatrix_1to0(pi/2,0,0,Q2);
    T_3to2 = TransformationMatrix_1to0(0,d2,0,Q3);
    T_4to3 = TransformationMatrix_1to0(0,d3,0,Q4+pi/2);
    
    Link_4_f0 = T_1to0*T_2to1*T_3to2*T_4to3*Link_4;
    Link_3_f0 = T_1to0*T_2to1*T_3to2*Link_3;
    Link_2_f0 = T_1to0*T_2to1*Link_2;

    %% Drawing
plotChessBoard();
hold on;
    colorLink = [ 83, 174, 203]/260;
    plot3([0;Link_2_f0(1)],[0;Link_2_f0(2)],[d1;Link_2_f0(3)],'-o',...
        [0;0],[0,0],[0,d1],'-o',...
        [Link_2_f0(1);Link_3_f0(1)],[Link_2_f0(2);Link_3_f0(2)],[Link_2_f0(3);Link_3_f0(3)],'-o',...
        [Link_3_f0(1);Link_4_f0(1)],[Link_3_f0(2);Link_4_f0(2)],[Link_3_f0(3);Link_4_f0(3)],'-o','color',colorLink,'LineWidth',2);
    grid on;
    zlabel('z (mm)');
    xlabel('x (mm)');
    ylabel('y (mm)');
    title("Robot arm visualization based on the requested end-effector pose");
    axis('equal');
    view(45,45);
end