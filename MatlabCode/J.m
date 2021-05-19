Establish Serial Connection
% Open the port to which the Arduino is connected and create a serial object.

% @ port                - COM port address of the Arduino. This will change depending on which USB port
%                   the arduino is connected to, and the exact structure of the address will vary between
%                   operating systems

% @ baudrate            - BaudRate used between MATLAB and Arduino, which is limited to a max of
%                   230400 by MATLAB.

% @ numID               - Number of detected motors.

% @ ID                  - Vector containing ID of each detected motor.

% @ establishSerial()   - A helper function that creates and returns a serial object while also performing
%                   a handshake with the Arduino to confim connection.

%%
% Specify COM Port
% port = '/dev/cu.usbmodem14131';                  % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
port = 'COM5';                                % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
% fclose(s);
s = establishSerial(port, baudrate);
% s = serialport(port, baudrate);

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)
% setControlMode(s, "velocity");

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;
%%
angleError = [-5,18,8,7]*pi/180;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NOTE: Remember to fclose(s) before disconnecting your Arduino!!! %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% call some neccessary function to work with, i.e forward Kinematic,
% jacobian, etc in form of symbolic, so that we can just subs the number in
% to save the computational cost and time.
%%
% forward Kinematics:
% r: end-effector pose in frame {0}
% T_eto0: Transformation mtx from e to 0
syms d1 d2 d3 d4 Q1 Q2 Q3 Q4 real;
[symsVar.r_E, symsVar.T_Eto0] = ForwardKinematic_EndEffector_to_0();


% jacobian matrix
[symsVar.J_inv,symsVar.J] = Jacobian_Matrix_ASM4();
%%

% Set motor EPROM to position control mode
setControlMode(s, "position");
%%
sendJointPos(s,[0, 0, pi/4],numID);

% sendJointPos(s,[0,gripper.openQ], 2);
%%
% setControlMode(s, "position");
sendJointPos(s, [-pi/20, 0, -pi/4, -pi/2, 0.5], numID);
%%
sendJointPos(s,[0,15*pi/180, -40*pi/180,-80*pi/180], numID);
% sendJointPos(s,[0/2 0/2], numID);
%%

readMotorFB(s,numID)*180/pi
readMotorFB(s,numID)
%%
double(substituteVariable(symsVar.r_E,fixAngleFromMotorFB(ans(1:4))))
%%
% test 4 corners for calibration
corner1 = [260;140;110]/1000; motorQ1 = fixAngleToMotorFB(InverseKinematicsGeneric(corner1));
corner2 = [260;-140;110]/1000; motorQ2 = fixAngleToMotorFB(InverseKinematicsGeneric(corner2));
corner3 = [540;140;180]/1000; motorQ3 = fixAngleToMotorFB(InverseKinematicsGeneric(corner3));
corner4 = [540;-140;180]/1000; motorQ4 = fixAngleToMotorFB(InverseKinematicsGeneric(corner4));
sendJointPos(s,[motorQ3+angleError,gripper.openQ], numID);

%%
% STEP 1: IMPORT ALL THE TRAJECTORY INTO WORKSPACE
% ---------------------------------------------------------------
% edit the move ID will gen the required square on chessboard

% listOfMove will gen the code of the square
% Square = listOfMove(moveID);

% motion_planning will return the required trajectory for the task
% expect running time: 10s
for moveID = 1:25
    trajectoryStruct.traj(moveID) = motion_planning_ASM4(listOfMove(moveID));
    fprintf("Done -- moveID --- " + string(moveID) + "\n");
end
%%
% CALCULATE ANGLE FOR HOME POSITION
Q           = InverseKinematicsGeneric([300;0;310]/1000);
homeMotorQ  = fixAngleToMotorFB(Q);

% define the angle of the grippper
gripper.closeQ  = 0.58;
gripper.openQ   = -0.65;
gripper.Q       = gripper.openQ; % as default

%%
GetBackHome(s, numID, homeMotorQ,gripper.closeQ, angleError);
angle = [homeMotorQ+angleError,gripper.openQ];
% sendJointPos(s,angle, numID);

%%
fffb = readMotorFB(s,numID)
(readMotorFB(s,numID))*180/pi
fixAngleFromMotorFB(fffb)
InverseKinematicsGeneric(r)

%%
fb = readMotorFB(s,numID);
for i = linspace(0,-pi/4,10)
    sendJointPos(s,[fb(1),i,fb(3),fb(4),fb(5)], numID);
    fb = readMotorFB(s,numID);
    plot(i,'.');hold on
    plot(fb(2));
end


%%
% Set Motor internal PID controller. Gains must be integers.
if (moveID ==1)
    tmpID = 1;      % motor id
    Kp = 7; Ki = 3; Kd = 1;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 10, 3, 0)       % motor 2
    setPID(s, 3, 5, 1, 0)       % motor 3
    setPID(s, 4, 5, 4, 0)       % motor 4
    setPID(s, 5, 5, 1, 0)
elseif (moveID ==2)
    tmpID = 1;      % motor id
    Kp = 7; Ki = 3; Kd = 2;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 7, 2, 0.1)       % motor 2
    setPID(s, 3, 7, 1, 0.1)       % motor 3
    setPID(s, 4, 10, 2, 0.1)       % motor 4
    setPID(s, 5, 10, 1, 0)
elseif (moveID == 3)
    tmpID = 1;      % motor id
    Kp = 7; Ki = 3; Kd = 1;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 7, 2.5, 1)       % motor 2
    setPID(s, 3, 7, 1, 0.1)       % motor 3
    setPID(s, 4, 5, 3, 0.5)       % motor 4
    setPID(s, 5, 10, 1, 0)
elseif (moveID == 4)
    tmpID = 1;      % motor id
    Kp = 7; Ki = 3; Kd = 1;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 8, 2.5, 1)       % motor 2
    setPID(s, 3, 8, .5, 1)       % motor 3
    setPID(s, 4, 4, 4, 0.5)       % motor 4
    setPID(s, 5, 10, 1, 0)
elseif (moveID == 5) % need to fix
    tmpID = 1;      % motor id
    Kp = 7; Ki = 7; Kd = 1;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 7, 2.5, 1)       % motor 2
    setPID(s, 3, 7 ,.5, 1)       % motor 3
    setPID(s, 4, 4, 4, 0.5)       % motor 4
    setPID(s, 5, 10, 1, 0)
elseif (moveID == 6)% not done yet
    tmpID = 1;      % motor id
    Kp = 7; Ki = 7; Kd = 1;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 7, 5, .5)       % motor 2
    setPID(s, 3, 5 ,2, .5)       % motor 3
    setPID(s, 4, 4, 4, 0.5)       % motor 4
    setPID(s, 5, 5, 5, 0)
elseif (moveID == 7)% not done yet
    tmpID = 1;      % motor id
    Kp = 7; Ki = 7; Kd = 1;
    setPID(s, tmpID, Kp, Ki, Kd)
    setPID(s, 2, 7, 5, .5)       % motor 2
    setPID(s, 3, 6,2, .5)       % motor 3
    setPID(s, 4, 4, 4, 0.5)       % motor 4
    setPID(s, 5, 10, 1, 0)
end

%% general PID tune for all cases
setPID(s, 1, 7, 7, 1)       % motor 1
setPID(s, 2, 7, 5.5, .5)       % motor 2
setPID(s, 3, 7, 2, .5)       % motor 3
setPID(s, 4, 4, 4, 1.5)       % motor 4
setPID(s, 5, 10, 1, 0)
%%
load trajectory_task1_asm4.mat
%%
% This is where you will write the majority of your code for part 2 of the project.
% get the data to control by edit the moveID
moveID      = 13;
trajectory = trajectoryStruct.traj(moveID);
%%
feedback.motor.Q    = [];
feedback.ref.Q      = [];
feedback.pose       = [];
gripper.Q       = gripper.openQ; % as default
for i = 1:2
    N   = length(trajectory.segment(i).poseRef(1,:));
    ref = trajectory.segment(i);
%     plot3(ref.poseRef(1,:),ref.poseRef(2,:),ref.poseRef(3,:),'-o','Color','r');
    for j = 1:N
%         Visualization_pose(ref.poseRef(:,j));
        r       = (ref.poseRef(:,j));
        Q       = InverseKinematicsGeneric(r);
        motorQ  = fixAngleToMotorFB(Q);
        fprintf("At i, j: %d, %d",i,j);
        fprintf("Ref pose: \n");r
        fprintf("Ref angle: \n");
        Q
        sendJointPos(s,[motorQ+angleError,gripper.Q], numID);
        pause(0.001);
        fprintf("Motor angle: \n");fb = readMotorFB(s,numID);
        fixAngleFromMotorFB(fb)
        
        % ______ print result for debugging _____

        feedback.ref.Q(:,j)     = Q';
        feedback.motor.Q(:,j) = fixAngleFromMotorFB(fb)'
        feedback.pose(:,j) = double(substituteVariable(symsVar.r_E,feedback.motor.Q(:,j)));
    end
%     pause(2);
    if (i == 1)
            closeGripper(s,gripper.openQ,gripper.closeQ, fb, numID);
            gripper.Q = gripper.closeQ;
    elseif (i == 2)
            openGripper(s,gripper.closeQ,gripper.openQ, fb, numID);
            gripper.Q = gripper.openQ;
    end
pause(3);
end

%%
figure(1);
plot3(feedback.pose(1,:),feedback.pose(2,:),feedback.pose(3,:),'-o','color','b');hold on
plot3(ref.poseRef(1,:),ref.poseRef(2,:),ref.poseRef(3,:),'-o','Color','r');hold on
% VisualizationJointSpace(feedback.motor.Q(:,j));

%%
for i = 1:4
    subplot(4,1,i);
    plot(feedback.ref.Q(i,:),'color','r','LineWidth',2);hold on;
    plot(feedback.motor.Q(i,:),'color','b','LineWidth',2);hold on;
end
%%
fb = readMotorFB(s,numID)
triggerTheGripper(s,fb(1:4),gripper.closeQ,numID);

readMotorFB(s,numID)
%%
fclose(s);

%%
% ------ ADDITIONAL CODE -----------

function value = substituteVariable(symVariable,jointDisp)
syms d1 d2 d3 d4 Q1 Q2 Q3 Q4 real;

[D1,D2,D3,D4] = RobotSpec_Assembly();
q1 = jointDisp(1);
q2 = jointDisp(2);
q3 = jointDisp(3);
q4 = jointDisp(4);

value = subs(symVariable, [d1,d2,d3,d4,Q1,Q2,Q3,Q4], [D1,D2,D3,D4,q1,q2,q3,q4]);

end

function [x,y,z]= GetOrientation(R)
x = R(1:3,1);
y = R(1:3,2);
z = R(1:3,3);
end

function orError = getOrientationError(ref,fb)
orError = 1/2*[cross(fb.Xe,ref.Xe) + cross(fb.Ye,ref.Ye) + cross(fb.Ze,ref.Ze)];
end

function newAngle = fixAngleFromMotorFB(angle)

% q1Zero = -pi;
q2Zero = -pi/2; % Joint displacement for sensorID = 2 to return to zero position
q3Zero = pi/4; % _____________________________= 3 __________________________
q4Zero = 0;
offset = [q2Zero, q3Zero, q4Zero];

newAngle(1) = -angle(1);
newAngle(2:4) = angle(2:4) - offset;
end

function newAngle = fixAngleToMotorFB(angle)
q1Zero = -pi;
q2Zero = -pi/2; % Joint displacement for sensorID = 2 to return to zero position
q3Zero = pi/4; % _____________________________= 3 __________________________
q4Zero = 0;
offset = [ q2Zero, q3Zero, q4Zero];

newAngle(1) = -angle(1);
newAngle(2:4) = angle(2:4) + offset;
end

function newVel = fixVelMotor(vel)
newVel = vel;
newVel(1) = -vel(1);

end

function feedback = readMotorFB(s,numID)
% Read motor position feedback
[feedback, eFB] = readFB(s, numID);
% feedback                                       % Display motor feedback
end

function vel = convertFromSym(symVel)
for t = 1:length(symVel)
    v = symVel(t,1);
    vel(t,1) = v.val;
end

end

function Delay(N)
for ttt = 1:N

end
end

function GetBackHome(s, numID, motorQ,gripper, error)
% This is where you will write a sequence to send all of your joints to their home positions.
% For home position, [Q1 Q2 Q3 Q4] = [0, pi/2, -pi/2, -pi/2]
sendJointPos(s,[motorQ+error,gripper], numID);
readMotorFB(s,numID);
end

function triggerTheGripper(s,feedback,gripperQ,numID)
sendJointPos(s,[feedback,gripperQ], numID);
end

function closeGripper(s,openQ,closeQ, feedback, numID)
% currentMotorFeedback = readMotorFB(s,numID);
for i = linspace(openQ,closeQ,5)
    sendJointPos(s,[feedback(1:end-1), i], numID);
%     pause(0.01);
end
end

function openGripper(s,closeQ,openQ, feedback, numID)
% currentMotorFeedback = readMotorFB(s,numID);
for i = linspace(closeQ,openQ,5)
    sendJointPos(s,[feedback(1:end-1), i], numID);
%     pause(0.01);
end
end
