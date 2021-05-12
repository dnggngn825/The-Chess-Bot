%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

% This function creates the transformation matrix T of frame {i} with
% respect to frame {i-1}


function T = TransformationMatrix_1to0(alpha, a, d, theta)

    R_x_0 = RotationTransformMatrix_1to0(alpha,'x');
    D_x_0 = TranslationMatrix([a;0;0]);
    D_z_1 = TranslationMatrix([0;0;d]);
    R_z_1 = RotationTransformMatrix_1to0(theta,'z');
    T = D_x_0 * R_x_0 * D_z_1 * R_z_1;

end