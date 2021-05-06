% Say Ee See 813641
% check TransformationMatrix_1to0 function is in path
% *********************************
% set arbitrary joint lengths
syms d1 d2 d3 d4 q1 q2 q3 q4 q1_dot q2_dot q3_dot q4_dot
% d1 = 200;
% d4 = 80;
line_width = 2;
[L1,L2,endEffectorGripLength,motorWidth,bigMotorRange, smallMotorRange, minAngle] = RobotSpec();
N = 40;
% 0 position Transformation matrices
T_1to0 = TransformationMatrix_1to0(0,0,d1,q1);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,q2);
T_3to2 = TransformationMatrix_1to0(0,d2,0,q3);
T_4to3 = TransformationMatrix_1to0(0,d3,0,pi/2+q4);
% for T_5to4 change between 0 and pi/2 to view zero and end-effector down position
T_5to4 = TransformationMatrix_1to0(pi/2,0,d4,0);
origin = [0;0;0;1];
%get end effector in frame 4
p5to4 = [0;-d4;0;1];

%TM to inertial frame
T_2to0 = T_1to0*T_2to1;
T_3to0 = simplify(T_2to0*T_3to2);
T_4to0 = simplify(T_3to0*T_4to3);
T_5to0 = simplify(T_4to0*T_5to4);
p1 = T_1to0*origin;
p2 = T_2to0*origin;
p3 = T_3to0*origin;
p4 = T_4to0*origin;
p5 = T_5to0*origin;
% p5 = simplify(T_1to0*T_2to1*T_3to2*T_4to3)*p5to4;
points = [origin,p1,p2,p3,p4,p5];
% z directions
z1 = T_1to0(1:3,3); z2 =  T_2to0(1:3,3); z3 =  T_3to0(1:3,3); z4 =  T_4to0(1:3,3);
% cross product skew matrices
sz_1 = [0 -1 0;1 0 0;0 0 0];
sz_2to4 = [0 0 -cos(q1);0 0 -sin(q1);cos(q1) sin(q1) 0];
% Jacobians
Jv = [sz_1*(p5(1:3)-p1(1:3)) sz_2to4*(p5(1:3)-p2(1:3)) sz_2to4*(p5(1:3)-p3(1:3)) sz_2to4*(p5(1:3)-p4(1:3))];
% Jv = [cross(z1,(p5(1:3)-p1(1:3))) cross(z2,(p5(1:3)-p2(1:3))) cross(z3,(p5(1:3)-p3(1:3))) cross(z4,(p5(1:3)-p4(1:3)))];
Jw = [z1 z2 z3 z4];
J_E = [Jv;Jw];
J_E = simplify(J_E);
J_E = simplify(vpa(J_E,2));
% define center of mass of joints
syms c1 c2 c3 c4
d = [d1,d2,d3,d4]; c = [c1,c2,c3,c4];
cm_2 = T_2to0*[c2;0;0;1]; 
cm_3 = T_3to0*[c3;0;0;1]; 
cm_4 = T_4to0*[0;-c4;0;1]; 
null_v = [0;0;0]; % vector for downstream joints
% distance from f1
p_11 = subs(p1(1:3),d(1),c(1)); 
p_12 = cm_2(1:3) - p1(1:3);
p_13 = cm_3(1:3) - p1(1:3); 
p_14 = cm_4(1:3) - p1(1:3);
% distance from f2
p_22 = cm_2(1:3) - p2(1:3)
p_23 = cm_3(1:3) - p2(1:3); 
p_24 = cm_4(1:3) - p2(1:3);
% distance from f3
p_33 = cm_3(1:3) - p3(1:3); 
p_34 = cm_4(1:3) - p3(1:3);
% distance from f4
p_44 = cm_4(1:3) - p4(1:3);

% Jacobian matrices for each joint
Jv1 = [sz_1*p_11 null_v null_v null_v];
Jw1 = [z1 null_v null_v null_v];
J_1 = [Jv1;Jw1]

Jv2 = [sz_1*p_12 sz_2to4*p_22 null_v null_v];
Jw2 = [z1 z2 null_v null_v];
J_2 = [Jv2;Jw2];
J_2 = simplify(J_2);
J_2 = vpa(J_2,2)

Jv3 = [sz_1*p_13 sz_2to4*p_23 sz_2to4*p_33 null_v];
Jw3 = [z1 z2 z3 null_v];
J_3 = [Jv3;Jw3];
J_3 = simplify(J_3);
J_3 = vpa(J_3,2)

Jv4 = [sz_1*p_14 sz_2to4*p_24 sz_2to4*p_34 sz_2to4*p_44];
Jw4 = [z1 z2 z3 z4];
J_4 = [Jv4;Jw4];
J_4 = simplify(J_4);
J_4 = vpa(J_4,2)

syms g M_1 M_2 M_3 M_4 M_E
m1=[0;0;-M_1*g];
m2=[0;0;-M_2*g];
m3=[0;0;-M_3*g];
m4=[0;0;-M_4*g];
G = [0;0;0;0];
G_test = (J_1(1:3,:)).'*m1 + (J_2(1:3,:)).'*m2...
                     + (J_3(1:3,:)).'*m3 + (J_4(1:3,:)).'*m4;
G_test = subs(G_test,[d1 d2 d3 d4 q1 q2 q3 q4 c1 c2 c3 c4 M_1 M_2 M_3 M_4 g],...
    [0.1 0.2 0.2 0.1 0 0 0 0 0.05 0.05 0.05 0.05 0.1 0.1 0.1 0.1 9.81]);
disp(G_test)

% test all torques 
Q4 = linspace(0,-pi/2,N);
Q3 = linspace(-minAngle,-3*pi/4,N);
Q2 = linspace(minAngle, bigMotorRange/2,N);
% Q1 = linspace(-bigMotorRange/2,bigMotorRange/2,N);
Q1 = 0;
%define masses
g = 9.81;
m1=[0;0;-0.067*g];
m2=[0;0;-0.111*g];
m3=[0;0;-0.111*g];
m4=[0;0;-0.027*g];
G = [0;0;0;0];

%% Find the max torque

% tmax1: is a 5x1 matrix where the first 4 rows presents the joint
% displacement for getting the max torque on joint 1, assigned in 
% tmax1(5). And similar to tmax2 3 and 4

tmax1 = zeros(5,1);
tmax2 = zeros(5,1);
tmax3 = zeros(5,1);
tmax4 = zeros(5,1);
h = Q1;
J_1 = subs(J_1,q1,h);
J_2 = subs(J_2,q1,h);
J_3 = subs(J_3,q1,h);
J_4 = subs(J_4,q1,h);

% sub test values
J_1 = subs(J_1,[d1 d2 d3 d4 c1 c2 c3 c4],[0.05 0.183 0.183 0.05 0.025 0.125 0.125 0.05]);
J_2 = subs(J_2,[d1 d2 d3 d4 c1 c2 c3 c4],[0.05 0.183 0.183 0.05 0.025 0.125 0.125 0.05]);
J_3 = subs(J_3,[d1 d2 d3 d4 c1 c2 c3 c4],[0.05 0.183 0.183 0.05 0.025 0.125 0.125 0.05]);
J_4 = subs(J_4,[d1 d2 d3 d4 c1 c2 c3 c4],[0.05 0.183 0.183 0.05 0.025 0.125 0.125 0.05]);
for k = Q2
        tempJ_2 = subs(J_2,q2,k);
        tempJ_3 = subs(J_3,q2,k);
        tempJ_4 = subs(J_4,q2,k);
        for j = Q3
        tempJ_3 = subs(tempJ_3,q3,j);
        tempJ_4 = subs(tempJ_4,q3,j);
        i = k - j + pi/2;
        tempJ_4 = subs(tempJ_4,q4,i);
        G = (J_1(1:3,:)).'*m1 + (tempJ_2(1:3,:)).'*m2...
             + (tempJ_3(1:3,:)).'*m3 + (tempJ_4(1:3,:)).'*m4;
            % compare to get largest vals
            if abs(G(1)) > abs(tmax1(5))
                tmax1(1) = h;
                tmax1(2) = k;
                tmax1(3) = j;
                tmax1(4) = i;
                tmax1(5) = G(1);
            end
            if abs(G(2)) > abs(tmax2(5))
                tmax2(1) = h;
                tmax2(2) = k;
                tmax2(3) = j;
                tmax2(4) = i;
                tmax2(5) = G(2);
            end
            if abs(G(3)) > abs(tmax3(5))
                tmax3(1) = h;
                tmax3(2) = k;
                tmax3(3) = j;
                tmax3(4) = i;
                tmax3(5) = G(3);
            end
            if abs(G(4)) > abs(tmax4(5))
                tmax4(1) = h;
                tmax4(2) = k;
                tmax4(3) = j;
                tmax4(4) = i;
                tmax4(5) = G(4);
            end
         end
end
T = [tmax1 tmax2 tmax3 tmax4];
disp(T);
fprintf("The torque generated by the mass of the arm is: \n");
disp(T);
fprintf("The minimum required torque the motor need to generate to overcome the mass of the arm is: \n");
disp(-T);
if (max(-T(5,:)) < 1.5)
    fprintf("The motor can produce enough torque to handle this\n");
else
    fprintf("Please revisit the design\n");
end


