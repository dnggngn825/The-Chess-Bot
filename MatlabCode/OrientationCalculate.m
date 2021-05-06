function [k_hat,phi_f] = OrientationCalculation(XI,XF)
R_i_0 = RotationMatrix_EEdown_Eto0(XI);
R_f_0 = RotationMatrix_EEdown_Eto0(XF);
R_f_i = R_i_0'*R_f_0;

% Rotation angle between 2 poses (rad)
phi_f = 2*acos(1/2*sqrt((1+ R_f_i(1,1) + R_f_i(2,2) + R_f_i(3,3))));

% Expression of k_hat in frame 0
k_hat = 1/(2*sin(phi_f))*[R_f_i(3,2)-R_f_i(2,3);R_f_i(1,3)-R_f_i(3,1);R_f_i(2,1)-R_f_i(1,2)];

end