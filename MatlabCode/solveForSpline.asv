function [arrayOfcoeffa, xAndvWTime] = solveForSpline(x0,xf,x0_dot,xf_dot,t_i,t_f)
syms a0 a1 a2 a3 real
syms t real

var = [a0,a1,a2,a3];
time = [1,t,t^2,t^3]';
time_dot = [1,2*t,3*t^2]';
xAndvWTime=[0,0,0];

eqns =[ x0 == var*subs(time,t_i), x0_dot == var(2:4)*subs(time_dot,t_i),xf == var*subs(time,t_f),xf_dot == var(2:4)*subs(time_dot,t_f)];
sol = solve(eqns,var);
arrayOfcoeffa = [sol.a0,sol.a1,sol.a2,sol.a3];
j = 1;
for i = linspace(t_i,t_f,3)
    xtime = subs(time,i);
    vtime = subs(time_dot,i);
    xAndvWTime(j,:) = [arrayOfcoeffa*xtime, arrayOfcoeffa(2:4)*vtime,i];  % x, displacement
    j= j +1;
end
end