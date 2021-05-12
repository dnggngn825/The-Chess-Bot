function R = RotationMatrix_EEdown_Eto0(X)

angle = atan2(X(2),X(1));
x_i_0 = [cos(angle);sin(angle);0];
y_i_0 = [cos(angle-pi/2);sin(angle-pi/2);0];
z_i_0 = [0;0;-1];

R = [x_i_0 y_i_0 z_i_0];
end