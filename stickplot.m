% creates a stick plot to verify forward kinematics
% Say Ee See 813641
% *********************************
% set arbitrary joint lengths

function stickplot()

d1 = 200;
d4 = 80;
line_width = 2;
% 0 position Transformation matrices
T_1to0 = TransformationMatrix_1to0(0,0,d1,0);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,0);
T_3to2 = TransformationMatrix_1to0(0,d1,0,0);
T_4to3 = TransformationMatrix_1to0(0,d1,0,pi/2);
T_5to4 = TransformationMatrix_1to0(pi/2,0,d4,0);
origin = [0;0;0;1];
p1 = T_1to0*origin;
p2 = T_1to0*T_2to1*origin;
p3 = T_1to0*T_2to1*T_3to2*origin;
p4 = T_1to0*T_2to1*T_3to2*T_4to3*origin;
p5 = T_1to0*T_2to1*T_3to2*T_4to3*T_5to4*origin;
points = [origin,p1,p2,p3,p4,p5];

%% Visualization - zero position
figure(1);
plot3(points(1,:),points(2,:),points(3,:),'-o');
axis([0 500 -100 100 0 300]);
dx = 10;
for i=1:6
    if i<=2
    text(points(1,i)+dx,points(2,i)+dx,points(3,i)+dx,['r' int2str(i)]);
    elseif i>3
    text(points(1,i)+dx,points(2,i)+dx,points(3,i)+dx,['r' int2str(i-1)]);
    end
end
grid on;
title("Configuration in zero position");
xlabel('x(mm)');
ylabel('y(mm)');
zlabel('z(mm)');
set(gcf,'color','w');

%% Configuration with end-effector pointing downwards
figure(2);
T_1to0 = TransformationMatrix_1to0(0,0,d1,0);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,0);
T_3to2 = TransformationMatrix_1to0(0,d1,0,0);
T_4to3 = TransformationMatrix_1to0(0,d1,0,0);
T_5to4 = TransformationMatrix_1to0(pi/2,0,d4,0);
origin = [0;0;0;1];
p1 = T_1to0*origin;
p2 = T_1to0*T_2to1*origin;
p3 = T_1to0*T_2to1*T_3to2*origin;
p4 = T_1to0*T_2to1*T_3to2*T_4to3*origin;
p5 = T_1to0*T_2to1*T_3to2*T_4to3*T_5to4*origin;
points = [origin,p1,p2,p3,p4,p5];

%% Visualization - configuration with end-effector pointing downwards
plot3(points(1,:),points(2,:),points(3,:),'-o');
axis([0 500 -100 100 0 300]);
dx = 10;
for i=1:6
    if i<=2
    text(points(1,i)+dx,points(2,i)+dx,points(3,i)+dx,['r' int2str(i)]);
    elseif i>3
    text(points(1,i)+dx,points(2,i)+dx,points(3,i)+dx,['r' int2str(i-1)]);
    end
end
grid on;
title("Configuration with end-effector pointing downwards");
xlabel('x(mm)');
ylabel('y(mm)');
zlabel('z(mm)');
set(gcf,'color','w');
end
