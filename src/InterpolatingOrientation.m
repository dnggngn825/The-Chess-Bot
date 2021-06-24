%% Robotics System Assignment 4 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

% this function is for interpolating the orientation for the trajectory
% including getting the phi_coefficients, phi_omega, k_hat and R with time


function [phi_coeff,phi_omega_time, k_hat,R_t_array] = InterpolatingOrientation(XI,XF,t_i,t_f)
    % 1. identify R_F_i
    R_i_0 = RotationMatrix_EEdown_Eto0(XI);
    R_f_0 = RotationMatrix_EEdown_Eto0(XF);
    R_f_i = R_i_0'*R_f_0;
    R_t_array = []; % R from t to 0
    
    % 2. calculate the phi_f
    phi_f = 2 * acos( 1/2 * sqrt(1 + R_f_i(1,1) + R_f_i(2,2) + R_f_i(3,3)));
    
    % 3. identify k_hat
    k_hat = 1/(2*sin(phi_f))*[R_f_i(3,2) - R_f_i(2,3);...
                            R_f_i(1,3) - R_f_i(3,1);...
                            R_f_i(2,1) - R_f_i(1,2)];
    
    % 4. interpolate phi(t) using cubic polynomial
    [phi_coeff, phi_omega_time] = solveForSpline(0,phi_f,0,0,t_i,t_f);
    
    % 5. convert into Euler parameters and get the orientation R
    for i = 1:length(phi_omega_time(:,1))
        e1 = k_hat(1)*sin(phi_omega_time(i,1)/2);
        e2 = k_hat(2)*sin(phi_omega_time(i,1)/2);
        e3 = k_hat(3)*sin(phi_omega_time(i,1)/2);
        e4 = cos(phi_omega_time(i,1)/2);
        
        R = [1-2*e2^2-2*e3^2 2*(e1*e2-e3*e4) 2*(e1*e3+e2*e4);...
            2*(e1*e2+e3*e4) 1-2*e1^2-2*e3^2 2*(e2*e3-e1*e4);...
            2*(e1*e3 -e2*e4) 2*(e2*e3 + e1*e4) 1-2*e1^2-2*e2^2];
        
        % omega, row, col
        R_t_array(i).matrix =R_i_0* R;
    end
    
end