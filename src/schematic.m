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
T_Eto0 = simplify(T_4to0*T_Eto4);

%% Produce the end-effector pose given trivial joint displacement using Forward Kinematics

% q = [pi/10; pi/3; -pi*3/4; 0];
q= [ 0 0 0 0]';

endEffectorPoseInframe4 = [0 -d4 0 1]';
endEffectorPoseInframeE = [0 0 0 1]';

T_4to0 = simplify(subs(T_4to0,{Q1,Q2,Q3,Q4},{q(1),q(2),q(3),q(4)}));

endEffectorPoseInframe0 = T_4to0 * endEffectorPoseInframe4;

% End effector pose for q = [0;0;0;0]
endEffectorPoseInframe0 = vpa(subs(endEffectorPoseInframe0, {d1,d2,d3,d4},{200,200,200,80}));

% FK Verification
stickplot_2();

function stickplot_2()
d1 = 115+68;
d4 = 50;
line_width = 3;

Q = [pi/8, pi/4,-2*pi/4, -pi/6];

% 0 position Transformation matrices
T_1to0 = TransformationMatrix_1to0(0,0,100,Q(1));
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,Q(2));
T_3to2 = TransformationMatrix_1to0(0,d1,0,Q(3));
T_4to3 = TransformationMatrix_1to0(0,d1,0,Q(4)+pi/2);
T_5to4 = TransformationMatrix_1to0(pi/2,0,d4,0);
T_2to0 =  T_1to0 * T_2to1;
z_1_0 = T_1to0(1:3,3);
z_2_0 = T_2to0(1:3,3);

origin = [0;0;0;1];
p1 = T_1to0*origin;
p2 = T_1to0*T_2to1*origin;
p3 = T_1to0*T_2to1*T_3to2*origin;
p4 = T_1to0*T_2to1*T_3to2*T_4to3*origin;
p5 = T_1to0*T_2to1*T_3to2*T_4to3*T_5to4*origin;
points = [origin,p1,p2,p3,p4,p5];

dx = 10;
figure(1);
z_1_0 = (70*z_1_0+p1(1:3));arrow3(p1(1:3)',z_1_0','k',70,5);
text(z_1_0(1)+dx,z_1_0(2)+dx,z_1_0(3)+dx,['z' int2str(1)]);hold on
z_2_0 = T_2to0(1:3,3); hold on;


z_2_0 = (70*z_2_0+p2(1:3));arrow3(p2(1:3)',z_2_0','k',5,70);
text(z_2_0(1)+dx,z_2_0(2)+dx,z_2_0(3)+dx,['z' int2str(2)]);hold on
z_2_0 = T_2to0(1:3,3); hold on;


z_2_0 = (70*z_2_0+p3(1:3));arrow3(p3(1:3)',z_2_0','k',5,70);
text(z_2_0(1)+dx,z_2_0(2)+dx,z_2_0(3)+dx,['z' int2str(3)]);hold on
z_2_0 = T_2to0(1:3,3);hold on;


z_2_0 = (70*z_2_0+p4(1:3));arrow3(p4(1:3)',z_2_0','k',5,70);
text(z_2_0(1)+dx,z_2_0(2)+dx,z_2_0(3)+dx,['z' int2str(4)]);hold on
z_2_0 = T_2to0(1:3,3);

hold on;

%% Visualization - zero position
dx = 10;
m2 = T_1to0*T_2to1*[150 0 0 1]';
m2 = m2(1:3);m2_2 = m2;m2_2(3) = m2(3)-100;
arrow3(m2',m2_2','r',50,5);hold on
text(m2_2(1)+dx,...
    m2_2(2)-dx,...
    m2_2(3)+dx,['m_2g']);
hold on

m3 = T_1to0*T_2to1*T_3to2*[150 0 0 1]';
m3 = m3(1:3);m3_2 = m3;m3_2(3) = m3(3)-100;
arrow3(m3',m3_2','r',50,5);hold on
text(m3_2(1)-2*dx,...
    m3_2(2)+dx,...
    m3_2(3)+dx,['m_3g']);
hold on

m4 = T_1to0*T_2to1*T_3to2*T_4to3*[0 -50 0 1]';
m4 = m4(1:3);m4_2 = m4;m4_2(3) = m4(3)-50;
arrow3(m4',m4_2','r',50,5);hold on
text(m4_2(1)+dx,...
    m4_2(2)-dx,...
    m4_2(3)+dx,['m_4g']);
hold on

plot3(points(1,:),points(2,:),points(3,:),'-o','LineWidth',line_width);
axis([0 500 -100 100 0 300]);

for i=1:6
    if i<=2
    text(points(1,i)+dx,points(2,i)-dx,points(3,i)+dx,['r' int2str(i)]);
    elseif i>3
    text(points(1,i)+dx,points(2,i)-dx,points(3,i)+dx,['r' int2str(i-1)]);
    end
end
hold on;
plot3(m3(1),m3(2),m3(3),'.','color','k','LineWidth',5);hold on
plot3(m2(1),m2(2),m2(3),'.','color','k','LineWidth',5);hold on
plot3(m4(1),m4(2),m4(3),'.','color','k','LineWidth',5);hold on
grid on;
title("Schematic of Robot Arm");
xlabel('x(mm)');
ylabel('y(mm)');
zlabel('z(mm)');
axis equal;
set(gcf,'color','w');
end

