%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

clear all;
clc;
%% Kit spec

[L1,L2,endEffectorGripLength,motorWidth,bigMotorRange, smallMotorRange, minAngle] = RobotSpec();


%% 2.2

r_3 = [L2;0;0;1];
r_2 = [L1;0;0;1];

x_1(1) = [0];
z_1(1) = [0];
N = 40;

x_0(1) = [0];
y_0(1) = [0];

% R_x_2 = RotationTransformMatrix_1to0(pi/6,'x');
% D_x_2 = TranslationMatrix([3.455;0;0]);
% D_z_3 = TranslationMatrix([1;2;6.32]);
% R_z_3 = RotationTransformMatrix_1to0(-pi/3,'z');

Q3 = linspace(-minAngle,-3*pi/4,N);
Q2 = linspace(minAngle, bigMotorRange/2,N);
Q1 = linspace(-bigMotorRange/2,bigMotorRange/2,N);

for k = Q1
T_1to0 = TransformationMatrix_1to0(0,0,0,k);

    for j = Q2

        T_2to1 = TransformationMatrix_1to0(pi/2,0,0,j);
        r_2_1 = T_2to1*r_2;
        r_2_0 = T_1to0*r_2_1;

        for i = Q3
            T_3to2 = TransformationMatrix_1to0(0,L1,0,i);
            r_3_2 = T_3to2*r_3;
            r_3_1 = T_2to1*r_3_2;
            x_1(end+1) = r_3_1(1);
            z_1(end+1) = r_3_1(3);
            [q1, q2, q3, q4] = InverseKinematics(r_3_1);
%             fprintf("Q2: %f, q2: %f,Q3: %f, q3: %f\n",j*180/pi,q2*180/pi,i*180/pi,q3*180/pi);
            
            r_3_0 = T_1to0*r_3_1;
            x_0(end+1) = r_3_0(1);
            y_0(end+1) = r_3_0(2);
        end
    end
end



%% Visualisation of chessboard

X_chess = 200;Y_chess = -117;
width_chess = 320;
heightFromJoint4toSurface = 120;

% Side view
figure(1)
plot(x_1,z_1,'.','color','k');
axis('equal');
line([X_chess X_chess+320],[Y_chess Y_chess],'color','b')
line([X_chess X_chess+320],[Y_chess+heightFromJoint4toSurface Y_chess+heightFromJoint4toSurface],'color','r')


% Top view
figure(2);
plot(x_0,y_0,'.','color','k');
axis('equal');

for t = 1:9
line([X_chess+width_chess*(t-1)/8 X_chess+width_chess*(t-1)/8],[-160 -160+width_chess],'color','b','LineWidth',2)
line([X_chess X_chess+width_chess],[-160+width_chess*(t-1)/8 -160+width_chess*(t-1)/8],'color','b','LineWidth',2)
end

