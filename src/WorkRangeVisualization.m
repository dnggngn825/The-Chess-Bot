%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen
% The link length can be adjusted in RobotSpec()

function WorkRangeVisualization()

[d1,d2,d3,d4,motorWidth,bigMotorRange, smallMotorRange, minAngle] = RobotSpec();
N = 15; % Change this to 75 for better visualization
Q1Range = pi/4;
Q4 = linspace(0, -pi/2,N);
Q3 = linspace(-minAngle,-3*pi/4,N);
Q2 = linspace(bigMotorRange/2, minAngle,N);
% Q1 = linspace(-bigMotorRange/2,bigMotorRange/2,N);
Q1 = linspace(-Q1Range,Q1Range,N);

endEffectorPoseInframe4 = [0 -d4 0 1]';
joint4PoseInFrame3 = [d3;0;0;1];

endEffectorPoseRange(1,1) = 0;endEffectorPoseRange(1,2) = 0;
joint4PoseRange(1,1) = 0;joint4PoseRange(1,2) = 0;

endEffectorPoseRangeSideView(1,1) = 0;endEffectorPoseRangeSideView(1,2) = d1;
joint4PoseRangeSideView(1,1) = 0;joint4PoseRangeSideView(1,2) = d1; % x and z axis in frame 1

endEffectorPoseRangeZ20(1,1) =  0;endEffectorPoseRangeZ20(1,2) =  0;
endEffectorPoseRangeZ90(1,1) =  0;endEffectorPoseRangeZ90(1,2) =  0;
gap = 2; % Allow a gap of +- 2mm to determine pose on xy plane at z = 20mm and 90mm


for k = Q1
T_1to0 = TransformationMatrix_1to0(0,0,d1,k);

    for j = Q2

        T_2to1 = TransformationMatrix_1to0(pi/2,0,0,j);

        for i = Q3
            T_3to2 = TransformationMatrix_1to0(0,d2,0,i);
            joint4PoseInFrame0 = T_1to0* T_2to1* T_3to2*joint4PoseInFrame3;
            joint4PoseInFrame1 = T_2to1* T_3to2*joint4PoseInFrame3;
            joint4PoseRange(end+1,:) = [joint4PoseInFrame0(1),joint4PoseInFrame0(2)];
            if (k == 0)
                joint4PoseRangeSideView(end+1,:) = [joint4PoseInFrame0(1),joint4PoseInFrame0(3)];
            end
            
                for t = max(-(pi/2-abs(abs(i)-abs(j))),-pi/2)
                    
                    T_4to3 = TransformationMatrix_1to0(0,d3,0,t+pi/2);
                    endEffectorPoseInframe0 =T_1to0* T_2to1* T_3to2* T_4to3 * endEffectorPoseInframe4;
                    endEffectorPoseRange(end+1,:) = [endEffectorPoseInframe0(1),endEffectorPoseInframe0(2)];
                    
                    if (endEffectorPoseInframe0(3) <= 20+gap) && ( endEffectorPoseInframe0(3) >= 20-gap)
                        endEffectorPoseRangeZ20(end+1,:) = [endEffectorPoseInframe0(1),endEffectorPoseInframe0(2)];
                    elseif (endEffectorPoseInframe0(3) <= 90+gap) && (endEffectorPoseInframe0(3) >= 90-gap)
                        endEffectorPoseRangeZ90(end+1,:) = [endEffectorPoseInframe0(1),endEffectorPoseInframe0(2)];
                    end
                    
                    endEffectorPoseInframe1 = T_2to1* T_3to2* T_4to3 * endEffectorPoseInframe4;
                    if (k == 0)
                        endEffectorPoseRangeSideView(end+1,:) = [endEffectorPoseInframe0(1),endEffectorPoseInframe0(3)];
                    end
                end
        end
    end
end

%% Visualisation of chessboard & Range of end-effector


[X_chess, Y_chess, width_chess, singleSquare]= ChessboardSpec();

% Top view
figure(3);
subplot(2,2,1);
plot(endEffectorPoseRangeZ90(:,1),endEffectorPoseRangeZ90(:,2),'.','color','k');
axis('equal');
xlabel('x (mm)');ylabel('y (mm)');
title('Top View of end-effector in XY plane at z = 90 mm');
% xlim([-100 400]);
% ylim([-250 250]);

for t = 1:9
line([X_chess+width_chess*(t-1)/8 X_chess+width_chess*(t-1)/8],[-width_chess/2 -width_chess/2+width_chess],'color','b','LineWidth',2)
line([X_chess X_chess+width_chess],[-width_chess/2+width_chess*(t-1)/8 -width_chess/2+width_chess*(t-1)/8],'color','b','LineWidth',2)
end

% figure(2);
subplot(2,2,2);
plot(endEffectorPoseRangeZ20(:,1),endEffectorPoseRangeZ20(:,2),'.','color','r');
axis('equal');
xlabel('x (mm)');ylabel('y (mm)');
title('Top View of end-effector in XY plane at z = 20 mm');
% xlim([-100 400]);
% ylim([-250 250]);

for t = 1:9
line([X_chess+width_chess*(t-1)/8 X_chess+width_chess*(t-1)/8],[-width_chess/2 -width_chess/2+width_chess],'color','b','LineWidth',2)
line([X_chess X_chess+width_chess],[-width_chess/2+width_chess*(t-1)/8 -width_chess/2+width_chess*(t-1)/8],'color','b','LineWidth',2)
end

% figure(4);
subplot(2,2,3);
plot(endEffectorPoseRangeSideView(:,1),endEffectorPoseRangeSideView(:,2),'.','color','k','LineWidth',4);
hold on;
plot([0;0;d2*cos(pi/6);d2*cos(pi/6)+d3*sin(75*pi/180);d2*cos(pi/6)+d3*sin(75*pi/180)],...
    [0;d1;d1+ d2*sin(pi/6);d1 + d2*sin(pi/6)-d3*cos(-75*pi/180);d1 + d2*sin(pi/6)-d3*cos(-75*pi/180)-d4],'-o','LineWidth',2);
hold on;
line([X_chess X_chess+width_chess],[Y_chess Y_chess],'color','b','LineWidth',2)
hold on;
plot([0;0;0;d3;d3],...
    [0;d1;d1+ d2;d1 + d2;d1 + d2-d4],'-o','color','m','LineWidth',2);
% line([X_chess X_chess+width_chess],[Y_chess+heightFromJoint4toSurface Y_chess+heightFromJoint4toSurface],'color','r')
xlabel('x (mm)');ylabel('z (mm)');
title('Working range of end-effector in frame 1 in XZ plane');
legend("Pose",  "Robot Arm",  "Chessboard",'Initial position');
% xlim([-100 400]);
% ylim([-200 400]);
axis('equal');


subplot(2,2,4);
plot([0;0;0;d3;d3],...
    [0;d1;d1+ d2;d1 + d2;d1 + d2-d4],'-o','color','m','LineWidth',2)
hold on;
axis('equal');hold on
line([X_chess X_chess+width_chess],[Y_chess Y_chess],'color','b','LineWidth',2)
xlabel('x (mm)');ylabel('z (mm)');
title('Initial position of the end-effector');
legend("Robot Arm", "Chessboard");
% xlim([-50 300]);
% ylim([-50 250]);



end