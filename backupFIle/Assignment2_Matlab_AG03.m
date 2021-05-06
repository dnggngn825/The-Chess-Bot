%% MCEN90028 ASSIGNMENT 2 MATLAB
% GROUP 3
% Written by: Dang Nguyen

%%

clear all;clc;
sympref('FloatingPointOutput',true);
syms d1 d2 d3 d4 Q1 Q2 Q3 Q4 real

T_1to0 = TransformationMatrix_1to0(0,0,d1,Q1);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,Q2);
T_3to2 = TransformationMatrix_1to0(0,d2,0,Q3);
T_4to3 = TransformationMatrix_1to0(0,d3,0,Q4+pi/2);
T_Eto4 = TransformationMatrix_1to0(pi/2,0,d4,0);

%% Transformation Matrix refer to frame 0

T_2to0 = T_1to0*T_2to1;
T_3to0 = simplify(T_2to0*T_3to2);
T_4to0 = simplify(T_3to0*T_4to3);
T_Eto0 = simplify(T_4to0 * T_Eto4);

%% Jv in frame 0

% Jv = [ z_i x r0E...]

r_01_0 = T_1to0(1:3,4);
r_02_0 = T_2to0(1:3,4);
r_03_0 = T_3to0(1:3,4);
r_04_0 = T_4to0(1:3,4);
r_0E_0 = T_Eto0(1:3,4);

r_1E_0 = r_0E_0 - r_01_0;
r_2E_0 = r_0E_0 - r_02_0;
r_3E_0 = r_0E_0 - r_03_0;
r_4E_0 = r_0E_0 - r_04_0;

R_iE_0 = [r_1E_0 r_2E_0 r_3E_0 r_4E_0];

% z axis of i in frame 0

z_1_0 = T_1to0(1:3,3);
z_2_0 = T_2to0(1:3,3);
z_3_0 = T_3to0(1:3,3);
z_4_0 = T_4to0(1:3,3);
z_E_0 = T_Eto0(1:3,3);

Z_i_0 = [z_1_0 z_2_0 z_3_0 z_4_0];

for i = 1:4
    Jv(:,i) = cross(Z_i_0(:,i),R_iE_0(:,i));
end

Jw = Z_i_0;
J = simplify([Jv;Jw]);

%% Verify Jv by taking derivative from r_OE_0

syms q1(t) q2(t) q3(t) q4(t) real;

new_r_0E_0 = subs(r_0E_0,[Q1,Q2,Q3,Q4],[q1,q2,q3,q4]);

new_r_0E_0_diff = diff(new_r_0E_0);

syms q1dot q2dot q3dot q4dot real;

pretty_new_r_0E_0_diff =  subs(new_r_0E_0_diff,[diff(q1),diff(q2),diff(q3),diff(q4)],[q1dot,q2dot,q3dot,q4dot]);

Jv_2nd = equationsToMatrix(pretty_new_r_0E_0_diff,[q1dot, q2dot,q3dot,q4dot]');
Jv_2nd = simplify(subs(Jv_2nd,[q1(t),q2(t),q3(t),q4(t)],[Q1,Q2,Q3,Q4]));

differenceMatrix = simplify(Jv_2nd-Jv);
if (differenceMatrix == zeros(3,4))
    fprintf("Correct Jv \n");
end

%% Jacobian matrix for torque

% Jvc1 = [cross(z_1_0,p1c1_0) [0;0;0;]];
% Jvc2 = [cross(z_1_0,p1c2) cross(z_2_0,p2c2)];


r_12_0 = r_02_0 - r_01_0;
r_13_0 = r_03_0 - r_01_0;
r_14_0 = r_04_0 - r_01_0;
r_23_0 = r_03_0 - r_02_0;
r_24_0 = r_04_0 - r_02_0;
r_34_0 = r_04_0 - r_03_0;

% c1,c2,c3,c4 are the CoM of d1,d2,d3,d4 respectively
syms c1 c2 c3 c4 real;

% mass of the related links and gravitational accel
syms m1 m2 m3 m4 g real;

% Derive the pose of CoM relative to joint 1 2 3 fram 0
p1c1_0 = [0; 0; -d1+c1];
p2c2_0 = subs(r_23_0,d2,c2);
p3c3_0 = subs(r_34_0,d3,c3);
p4c4_0 = subs(r_4E_0,d4,c4);
p1c2_0 = r_02_0 + p2c2_0;
p1c3_0 = r_13_0 + p3c3_0;
p1c4_0 = r_14_0 + p4c4_0;
p2c3_0 = r_23_0 + p3c3_0;
p2c4_0 = r_24_0 + p4c4_0;
p3c4_0 = r_34_0 + p4c4_0;
%% Find the max torque across the workspace
ass2_Jderv();

%% archieve
% % Derive Jacobian matrix associated with translational velocities of the
% % CoM
% Jvc1 = [cross(z_1_0,p1c1_0) zeros(3,3)];
% Jvc2 = simplify([cross(z_1_0,p1c2_0) cross(z_2_0,p2c2_0) zeros(3,2)]);
% Jvc3 = simplify([cross(z_1_0,p1c3_0) cross(z_2_0,p2c3_0) cross(z_3_0,p3c3_0) zeros(3,1)]);
% Jvc4 = simplify([cross(z_1_0,p1c4_0) cross(z_2_0,p2c4_0) cross(z_3_0,p3c4_0) cross(z_4_0,p4c4_0)]);
% 
% % G matrix
% g1 = [0 0 -m1*g]';
% g2 = [0 0 -m2*g]';
% g3 = [0 0 -m3*g]';
% g4 = [0 0 -m4*g]';
% 
% G = Jvc1'*g1 + Jvc2'*g2 + Jvc3'*g3 + Jvc4'*g4;
% % 
% % subs(G,[d1 d2 d3 d4 Q1 Q2 Q3 Q4 c1 c2 c3 c4 m1 m2 m3 m4 g],...
% %            [0.1 0.1 0.1 0.1 0 0 0 0 0.05 0.05 0.05 0.05 0.1 0.1 0.1 0.1 9.81])


% 
% maxTorque = zeros(4,1);
% 
% [D1,D2,D3,D4,motorWidth,bigMotorRange, smallMotorRange, minAngle] = RobotSpec();
% N = 15; % Change this to 75 for better visualization
% q1Range = pi/4;
% q4 = linspace(0, -pi/2,N);
% q3 = linspace(-minAngle,-3*pi/4,N);
% q2 = linspace(bigMotorRange/2, minAngle,N);
% q1 = linspace(-q1Range,q1Range,N);

% Center of mass position
% CoM = 0.05; % in meters

% for i = q1
%     for j = q2
%         for k = q3
%             for l =q4
%                 torque = subs(G,[d1 d2 d3 d4 Q1 Q2 Q3 Q4 c1 c2 c3 c4 m1 m2 m3 m4 g],...
%                                         [D1 D2 D3 D4 i j k l CoM CoM CoM CoM 0.1 0.1 0.1 0.1 9.81]);
%                 maxTorque = [min(torque(1),maxTorque(1));
%                                       min(torque(2),maxTorque(2));
%                                       min(torque(3),maxTorque(3));
%                                       min(torque(4),maxTorque(4));];
%             end
%         end
%     end
% end

% disp(maxTorque);
% subs(G,[d1 d2 d3 d4 Q1 Q2 Q3 Q4 c1 c2 c3 c4 m1 m2 m3 m4 g],...
%             [0.1 0.2 0.2 0.1 0 0 0 0 CoM CoM CoM CoM 0.1 0.1 0.1 0.1 9.81])

