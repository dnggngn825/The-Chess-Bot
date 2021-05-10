%% Robotics System Assignment 1 Matlab Code

%   The University of Melbourne
%   MCEN90028 ROBOTICS SYSTEMS
%   GROUP 3
%   Written by: Hai Dang Nguyen

%%
% r: the end-effector pose expressed in the inertial frame
% r will be in frame {0}, it will then be converted to frame {1} where all
% of the calculation will be performed.

function qFinalSolution = InverseKinematicsGeneric(r)
Q3 = sym('Q3');


% r1 = [175;0 ;20;1];
% r2 = [110;65 ;20;1];
% r= r2;

[d1,d2,d3,d4,motorWidth,bigMotorRange, smallMotorRange, minAngle] = RobotSpec();
[d1,d2,d3,d4] =  RobotSpec_Assembly();

% Solve for Q1
q1 = atan(r(2)/r(1));
    
T_1to0 = TransformationMatrix_1to0(0,0,d1,q1);
T_0to1 = inv(T_1to0);
% endEffectorPoseInFrame1 = T_0to1*r; % Convert to frame {1}

% Solve for Q3 using cosine rule
Joint4Pose_E = [0 0 -d4 1]';
R_Eto0 = [cos(q1) sin(q1) 0;...
                sin(q1) -cos(q1) 0;...
                0 0 -1];
T_Eto0 = [R_Eto0 r(1:3); 0 0 0 1];
Joint4Pose_1 = T_0to1*T_Eto0*Joint4Pose_E;
xe = Joint4Pose_1(1);ze = Joint4Pose_1(3);
eqn1 = xe^2 + ze^2 - d2^2 -d3^2 + 2*d2*d3*cos(pi-Q3) == 0;
Q3sol = solve(eqn1,Q3);
Q3sol = double(Q3sol);
    for i = 1:length(Q3sol)
        Q3sol(i) = withIn180(Q3sol(i));
    end
Q3sol = 2*pi*(abs(Q3sol)>pi) -Q3sol;


% Solve for Q2
Q2 = sym('Q2');
Q2sol(:,1) = [0];
q = [];
qFinalSolution = [];

    for i = 1:length(Q3sol)
    %     eqn2 = d2 *cos(Q2) + d3 * cos(Q3sol(i)+Q2) - xe == 0;
        eqn2 = Q2 - atan2(ze,xe) - asin(d3*sin(pi+Q3sol(i))/sqrt(xe^2+ze^2)) == 0;
        sol = solve(eqn2, Q2);
        Q2sol(:,i) = double(sol);
        new_q = [Q2sol(:,i)';Q3sol(i)*ones(size(Q2sol(:,i)'))];
        q = [q new_q];
    end

% Solve for Q4
    if (~isempty(q))
        for i = 1:length(q(1,:))
            q2 = q(1,i);q3 = q(2,i);

            T_2to1 = TransformationMatrix_1to0(pi/2,0,0,q2);
            T_3to2 = TransformationMatrix_1to0(0,d2,0,q3);
        %     T_4to3 = TransformationMatrix_1to0(0,d3,0,Q4);
            T_Eto4 = TransformationMatrix_1to0(pi/2,0,d4,0);

    %         q4sol = [-(pi/2 - (2*((q3<0 && q2>0)-0.5))*(abs(q3) - abs(q2)))];
            q4sol = -(q2+q3+pi/2);


            for t= 1:length(q4sol(:,1))
                T_4to3 = TransformationMatrix_1to0(0,d3,0,q4sol(t,1)+pi/2);
                EEFK(:,i) = real(T_1to0 * T_2to1 * T_3to2 * T_4to3 *T_Eto4) * [0;0;0;1];

    %             if (abs(EEFK - r) < [10^-2;10^-2;10^-2;10^-2])
    %                 Visualization(q1,q2,q3,q4sol(t,1));
                    qFinalSolution = real([qFinalSolution;q1,q2,q3,q4sol(t,1)]);
    %             end
            end
        end
    else
       fprintf("There is no solution, as it's out of range\n");
       Visualization(q1,minAngle,-minAngle,-pi/2);
       qFinalSolution = [q1,minAngle,-minAngle,-pi/2];
    end
    
    %% Take the correct solution
    
    filter_Q = [];
    for i = 1: length(qFinalSolution(:,1))
        if (qFinalSolution(i,2) >= 0 && qFinalSolution(i,3) <= 0 && qFinalSolution(i,4) <= 0)
            filter_Q(end+1,:) = qFinalSolution(i,:);
        end
    end
    qFinalSolution = filter_Q;

function angle = withIn180(angleIn)

    if ((abs(angleIn) > pi) && (abs(angleIn) < 2*pi))
        angle = (-2*((angleIn > 0)-0.5)).*(2*pi*ones(size(angleIn)) - abs(angleIn));
    elseif (abs(angleIn) > 2*pi)
        angle = 2*((angleIn > 0)-0.5).*(abs(angleIn)-2*pi*ones(size(angleIn)));
    else
        angle = angleIn;
    end
end

end
