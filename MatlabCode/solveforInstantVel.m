function xref =  solveforInstantVel(x0,xf,v0, vf,t_i,t_f)
xref = [solveForSplineOne(x0(1),xf(1),v0(1),vf(1),t_i,t_f);...
    solveForSplineOne(x0(2),xf(2),v0(2),vf(2),t_i,t_f);...
    solveForSplineOne(x0(3),xf(3),v0(3),vf(3),t_i,t_f);...
    InterpolatingOrientation(x0,xf,t_i,t_f)];

    function [v] =  solveForSplineOne(x0,xf,x0_dot,xf_dot,t_i,t_f)
        a0 = sym('a0');a1 = sym('a1');a2 = sym('a2');a3 = sym('a3');
        t = sym('t');

        var = [a0,a1,a2,a3];
        time = [1,t,t^2,t^3]';
        time_dot = [1,2*t,3*t^2]';

        eqns =[ x0 == var*subs(time,t_i), x0_dot == var(2:4)*subs(time_dot,t_i),xf == var*subs(time,t_f),xf_dot == var(2:4)*subs(time_dot,t_f)];
        sol = solve(eqns,var);
        arrayOfcoeffa = [sol.a0,sol.a1,sol.a2,sol.a3];
        
       for i = linspace(t_i,t_f,3)
            vtime = subs(time_dot,i);
            if ((i ~= t_i) && (i~= t_f))
                v = arrayOfcoeffa(2:4)*vtime;
            end
        end
    end

    function [omega] = InterpolatingOrientation(XI,XF,t_i,t_f)
        % 1. identify R_F_i
        R_i_0 = RotationMatrix_EEdown_Eto0(XI);
        R_f_0 = RotationMatrix_EEdown_Eto0(XF);
        R_f_i = R_i_0'*R_f_0;

        % 2. calculate the phi_f
        phi_f = 2 * acos( 1/2 * sqrt(1 + R_f_i(1,1) + R_f_i(2,2) + R_f_i(3,3)));

        % 3. identify k_hat
        k_hat = 1/(2*sin(phi_f))*[R_f_i(3,2) - R_f_i(2,3);...
                                R_f_i(1,3) - R_f_i(3,1);...
                                R_f_i(2,1) - R_f_i(1,2)];

        % 4. interpolate phi(t) using cubic polynomial
        [thetadot] = solveForSplineOne(0,phi_f,0,0,t_i,t_f);
        omega = thetadot*k_hat;

        % 5. convert into Euler parameters and get the orientation R
%         for i = 1:length(phi_omega_time(:,1))
%             e1 = k_hat(1)*sin(phi_omega_time(i,1)/2);
%             e2 = k_hat(2)*sin(phi_omega_time(i,1)/2);
%             e3 = k_hat(3)*sin(phi_omega_time(i,1)/2);
%             e4 = cos(phi_omega_time(i,1)/2);
% 
%             R = [1-2*e2^2-2*e3^2 2*(e1*e2-e3*e4) 2*(e1*e3+e2*e4);...
%                 2*(e1*e2+e3*e4) 1-2*e1^2-2*e3^2 2*(e2*e3-e1*e4);...
%                 2*(e1*e3 -e2*e4) 2*(e2*e3 + e1*e4) 1-2*e1^2-2*e2^2];
% 
%             % omega, row, col
%             R_t_array(i).matrix =R_i_0* R;
%         end
    end
end