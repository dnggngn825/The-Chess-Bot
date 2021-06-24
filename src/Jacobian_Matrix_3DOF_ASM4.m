function [Jv, r_4] = Jacobian_Matrix_3DOF_ASM4()
sympref('FloatingPointOutput',true);
syms d1 d2 d3 d4 Q1 Q2 Q3 Q4 real

T_1to0 = TransformationMatrix_1to0(0,0,d1,Q1);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,Q2);
T_3to2 = TransformationMatrix_1to0(0,d2,0,Q3);
T_4to3 = [eye(3),[d3;0;0];0 0 0 1];

%% Transformation Matrix refer to frame 0

T_2to0 = T_1to0*T_2to1;
T_3to0 = simplify(T_2to0*T_3to2);
T_4to0 = simplify(T_3to0*T_4to3);
% T_Eto0 = simplify(T_4to0 * T_Eto4);
R_1to0 = T_1to0(1:3,1:3);
R_2to0 = T_2to0(1:3,1:3);

%% Jv in frame 0

% Jv = [ z_i x r0E...]

r_01_0 = T_1to0(1:3,4);
r_02_0 = T_2to0(1:3,4);
r_03_0 = T_3to0(1:3,4);
r_04_0 = T_4to0(1:3,4);
% r_0E_0 = T_Eto0(1:3,4);
r_4 = r_04_0;

r_1E_0 = r_04_0 - r_01_0;
r_2E_0 = r_04_0 - r_02_0;
r_3E_0 = r_04_0 - r_03_0;
% r_4E_0 = r_0E_0 - r_04_0;

R_iE_0 = [r_1E_0 r_2E_0 r_3E_0];
R_iE_1 = simplify(R_1to0'*R_iE_0);
R_iE_2 = simplify(R_2to0'*R_iE_0);

% z axis of i in frame 0

z_1_0 = T_1to0(1:3,3);
z_2_0 = T_2to0(1:3,3);
z_3_0 = T_3to0(1:3,3);
z_4_0 = T_4to0(1:3,3);
% z_E_0 = T_Eto0(1:3,3);

Z_i_0 = [z_1_0 z_2_0 z_3_0];
Z_i_1 = simplify(R_1to0' * Z_i_0);
Z_i_2 = simplify(R_2to0' * Z_i_0);

for i = 1:3
    Jv(:,i) = cross(Z_i_0(:,i),R_iE_0(:,i));
    Jv_1(:,i) = cross(Z_i_1(:,i),R_iE_1(:,i));
    Jv_2(:,i) = cross(Z_i_2(:,i),R_iE_2(:,i));
end

% Jw = Z_i_0;
% Jw_1 = Z_i_1;
% Jw_2 = Z_i_2;
% J = simplify([Jv;Jw]);
% J_1 = simplify([Jv_1;Jw_1]);

end