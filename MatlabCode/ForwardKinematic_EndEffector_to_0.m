function [r_E, T_Eto0] = ForwardKinematic_EndEffector_to_0()

% r: the vector indicating the position of the end-effector [3x1]
% Q_set: the set of joint displacement to feed into FK

syms d1 d2 d3 d4 Q1 Q2 Q3 Q4 real

% Calculate the transformation matrices from DH table
T_1to0 = TransformationMatrix_1to0(0,0,d1,Q1);
T_2to1 = TransformationMatrix_1to0(pi/2,0,0,Q2);
T_3to2 = TransformationMatrix_1to0(0,d2,0,Q3);
T_4to3 = TransformationMatrix_1to0(0,d3,0,Q4+pi/2);
T_Eto4 = TransformationMatrix_1to0(pi/2,0,d4,0);
T_4to0 = T_1to0 * T_2to1 * T_3to2 * T_4to3;
T_Eto0 = (T_4to0*T_Eto4);
T_Eto0 = simplify(T_Eto0);

% end-effector pose
r = T_Eto0 * [0;0;0;1];

r_E = r(1:3);
r_E = simplify(r_E);

end