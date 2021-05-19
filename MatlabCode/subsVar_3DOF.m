function value = subsVar_3DOF(symVariable,jointDisp)
syms d1 d2 d3 d4 Q1 Q2 Q3 real;

[D1,D2,D3,D4] = RobotSpec_Assembly();
q1 = jointDisp(1);
q2 = jointDisp(2);
q3 = jointDisp(3);
% q4 = jointDisp(4);

value = double(subs(symVariable, [d1,d2,d3,d4,Q1,Q2,Q3], [D1,D2,D3,D4,q1,q2,q3]));

end