% ChessBot_Skeleton_Code.mlx
% --------------------------
% Licenting Information: You are free to use or extend this project
% for educational purposes provided that (1) you do not distribute or
% publish solutions, (2) you retain this notice, and (3) you provide
% clear attribution to the University of Melbourne, Department of
% Mechanical Engineering.
%
% Attribution Information: The ChessBot project was developed at the
% University of Melbourne. The core project was primarily developed
% by Professor Denny Oetomo (doetomo@unimelb.edu.au). The ChessBot
% Skeleton Code was developed by Nathan Batham
% (nathan.batham@unimelb.edu.au)



How To Use This Skeleton Code

% Set COM Port          - Under the "Establish Serial Connection" section, select
%               the appropriate port format and identifier. This should be the
%               same as what is shown in the Arduino IDE software under 'Tools ->
%               Port'.

% Executing Section     - To execute a single section this Live Script, click into
%               the section, so that it is highlighted on the left hand side. Then,
%               press ctrl+enter (or cmd+enter on mac).

% Sample Code           - The functions provided have been designed as a guide for
%               basic functionality only. It is advised to expand them and dig into
%               the code to truely understand the the motor capabilities. It is also
%               recommended to remove the sections in this file labelled "Sample Code"
%               once you have become familiar with how each operation works.

% Position Feedback     - The feedback from the provided motors is only betwen -pi/2
%               and +pi/2. As such, it is NOT ADVISED to use gearboxes or any form
%               of mechanical reduction. This will also limit your reachable workspace.
%               In the Arduino code, the vector "motorOffset" contains the offset used
%               to set the range of motion so that the centre is at 0. This can be
%               modified if required to increase motion in one direction or the other,
%               which may help with some reachable workspace and home position issues.


% Closing Serial        - IMPORTANT: The serial connection MUST be closed before either
%               a) disconnecting the Arduino, or b) clearing the serial object.
%               Failure to do so may result in the COM port becoming inaccessible.
%               In the instance that this does occur, a full system restart will
%               be required. Please see fclose() line at the end of this file and
%               execute whenever necessary.

% BUG REPORTING         - If you believe there may be a bug in the code, please report
%               it using the subject discussion board. Any revisions will be uploaded
%               to CANVAS and all students will be notified.

%%
% Establish Serial Connection
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


% Specify COM Port
% port = '/dev/cu.usbmodem14131';                  % MacOS Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS
port = 'COM5';                                % Windows Port Structure - ONLY EXAMPLE, MAY NOT BE EXACT ADDRESS

% Setup Serial Connection
baudrate = 230400;
fclose(s);
s = establishSerial(port, baudrate);
% s = serialport(port, baudrate);

% Read Connected Motor IDs
[numID, ID] = getMotorIDs(s)
% setControlMode(s, "velocity");

% Set Change Motor ID flag - CHANGE TO ADJUST MOTOR ID
changeMotorID = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NOTE: Remember to fclose(s) before disconnecting your Arduino!!! %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%
angleError = [0,20,7,10]*pi/180;

% Read joystick input
[xJoy, yJoy, eJoy] = readJoy(s);
[xJoy, yJoy]                                    % Display joystick input

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
% CALCULATE ANGLE FOR HOME POSITION
Q           = InverseKinematicsGeneric([300;0;310]/1000);
homeMotorQ  = fixAngleToMotorFB(Q);

% define the angle of the grippper
gripper.closeQ  = 0.18;
gripper.openQ   = -0.65;
gripper.Q       = gripper.openQ; % as default

%%
% Home Robot
setControlMode(s, "position");  

%%
setControlMode(s, "velocity");

sendJointVel(s, [round(double(Q_cmd_dot_motor'),3), 0], numID);

%%
% This is where you will write a sequence to send all of your joints to their home positions.
% For home position, [Q1 Q2 Q3 Q4] = [0, pi/2, -pi/2, -pi/2]
Q       = InverseKinematicsGeneric([320;0;310]/1000);
motorQ  = fixAngleToMotorFB(Q);
%%
GetBackHome(s, numID, motorQ, gripper.openQ, angleError);
readMotorFB(s,numID)
(readMotorFB(s,numID))*180/pi
%%

% Task space velocity controller
movementType = 'x';
if (movementType == 'x')
%     controller.Kp = 0.02*diag([2, 5, 2, 10, 25, 10]) ;      % tr (x, y, z), orie (x, y, z)
%         controller.Kp = 0.04*diag([0, 5, 0, 0, 25, 10]) ; controller.Ki = 0.3*controller.Kp;        % stable but easily pose to disturbances
        controller.Kp = 0.01*[2, 2, 1, 2, 4, 4]';         

elseif (movementType == 'y')
    controller.Kp = 1*diag([0.75, 0, 0.25, 1, 1, 1]);       % tr (x, y, z), orie (x, y, z)
end

% controller.Ki = 0.005* diag([0.5, 2, 2, 1, 0.1, 1]);
% controller.Ki = 0 * eye(6);
controller.Ki = 0.03*0.01*[6, 2, 4, 2, 4, 4]';
Incontroller.Kp = 0.1*diag([0.1, 0.1, 0.1, 0.1]);
Incontroller.Ki = 0.2*Incontroller.Kp;

%%
setControlMode(s, "position");

%%
current.pose = [320;0;150]/1000;
Q       = InverseKinematicsGeneric(current.pose);
motorQ  = fixAngleToMotorFB(Q);
GetBackHome(s, numID, motorQ, gripper.openQ, angleError);
readMotorFB(s,numID)
(readMotorFB(s,numID))*180/pi

%%
% main script
refPose = substituteVariable(symsVar.r_E,fixAngleFromMotorFB(readMotorFB(s,numID)));
fprintf("Ref Pose: ");refPose
% refPose         = [260;0;150]/1000;
currentPose     = refPose;
feedback.RMtrx  = [1 0 0; 0 -1 0; 0 0 -1];
Kp = controller.Kp;
plotPose.pose   = [];plotPose.poseError = [];
plotPose.velCmd = [];
plotPose.Q      = [];
setControlMode(s, "velocity");
previousError = zeros(6,1);
counter = 0;
QerrorPrev      = 0;
QerrorCurrent      = 0;Q_cmd_dot = 0;
plotPose.in.Qerror = [];Q_dotCmd = zeros(4,1);
plotPose.orErr = [];

while (counter < 200)
    N=10;
    tic;
    for i = 1:N
    % motion planning
    feedbackQ = fixAngleFromMotorFB(readMotorFB(s,numID));
    currentPose = substituteVariable(symsVar.r_E,feedbackQ);
    fprintf("Current pose: ");currentPose
    feedback.RMtrx = substituteVariable(symsVar.T_Eto0(1:3,1:3),feedbackQ);
    
    % gen the x_ref_dot
    dt_refPoint          = toc; tic;
    %     x_ref_dot            = solveforInstantVel(currentPose,refPose,[0,0,0],[0,0,0],0,dt);
    x_ref_dot = zeros(6,1);
    
    % get the error in pose
    poseError = refPose - currentPose;
    plotPose.poseError(:,end+1) = poseError;
    fprintf("Pose error:");poseError'
    % get orientation error
    ref.Xe = [1;0;0];
    ref.Ye = [0;-1;0];
    ref.Ze = [0;0;-1];
    [feedback.Xe, feedback.Ye, feedback.Ze] = GetOrientation(feedback.RMtrx);
    orError = getOrientationError(ref,feedback);
    plotPose.orErr(:,end+1) = orError;
    % plot
    plotPose.pose(:,end+1)   = currentPose;
    plotPose.Q(:,end+1)      = feedbackQ';
    
    fprintf("Sampling time: ");
        dt_controller = dt_refPoint
        % feed in the controller
%         x_cmd_dot =  x_ref_dot + Kp*([poseError;orError] - previousError) + ...
%             controller.Ki*dt_controller/2*([poseError;orError] + previousError);
        
        % ---- controller -----
        x_cmd_dot(1) =  x_ref_dot(1) + Kp(1)*(poseError(1) - previousError(1)) + ...
            controller.Ki(1)*dt_controller/2*(poseError(1) + previousError(1));
        x_cmd_dot(2) =  x_ref_dot(2) + Kp(2)*(poseError(2) - previousError(2)) + ...
            controller.Ki(2)*dt_controller/2*(poseError(2) + previousError(2));
        x_cmd_dot(3) =  x_ref_dot(3) + Kp(3)*(poseError(3) - previousError(3)) + ...
            controller.Ki(3)*dt_controller/2*(poseError(3) + previousError(3));
        x_cmd_dot(4) =  x_ref_dot(4) + Kp(4)*(orError(1) - previousError(4)) + ...
            controller.Ki(4)*dt_controller/2*(orError(1) + previousError(4));
        x_cmd_dot(5) =  x_ref_dot(5) + Kp(5)*(orError(2) - previousError(5)) + ...
            controller.Ki(5)*dt_controller/2*(orError(2) + previousError(5));
        x_cmd_dot(6) =  x_ref_dot(6) + Kp(6)*(orError(3) - previousError(6)) + ...
            controller.Ki(6)*dt_controller/2*(orError(3) + previousError(6));
        %------

        
        % convert to Q_cmd_dot
        controller.JMtrx = double(substituteVariable(symsVar.J,feedbackQ));
%         invJMtrx         = controller.JMtrx'/(controller.JMtrx*controller.JMtrx');
        fprintf("Current angle: ");feedbackQ
        try
            invJMtrx = substituteVariable(symsVar.J_inv,feedbackQ);
        catch ME
            invJMtrx = controller.JMtrx'/(controller.JMtrx*controller.JMtrx');
        end
        Q_cmd_dot_prev = Q_cmd_dot;
        Q_cmd_dot        = double(invJMtrx)*round(double(x_cmd_dot),3);
        Q_cmd_dot_motor  = fixVelMotor(double(Q_cmd_dot));
        fprintf("Q cmd dot");Q_cmd_dot'
        
        % ======= Inner Loop ==========
%         in = tic;
%         for k = 1:10
%             fprintf("Start inner loop: ========================\n")
%             QerrorPrev      = QerrorCurrent;
%             QerrorCurrent = Q - fixAngleFromMotorFB(readMotorFB(s,numID));
%             plotPose.in.Qerror(:,end+1) = QerrorCurrent;
%             
%             fprintf("Inner error Q: \n");QerrorCurrent
%             
%             % control term
%             dt = toc(in); in = tic;
%             Q_cmd_dot_prev = Q_dotCmd;
%             Q_dotCmd      = Q_cmd_dot_prev + Q_cmd_dot - Q_cmd_dot_prev + Incontroller.Kp* (QerrorCurrent' - QerrorPrev') + ...
%                                         Incontroller.Ki * dt/2 *(QerrorCurrent' + QerrorPrev') ;
%             fprintf("Inner loop sampling time: \n");
%             dt
%             fprintf("Inner loop ang vel cmd: \n");
%             fixVelMotor(Q_dotCmd)'
%             % need to convert Q dot for motor
%             sendJointVel(s, round(fixVelMotor(double(Q_dotCmd)),3), numID);
%             plotPose.velCmd(:,end+1) = round(double(Q_dotCmd'),3);
% 
%             % a delay for it to run
% %             pause(dt_innerLoop);s
% 
% %             innerLoop.feedback.Q = fixAngleFromMotorFB(readMotorFB(s, 4));
% %             innerLoop.error.prev = innerLoop.error.current;
% %             fprintf("Inner Q fb: \n");innerLoop.feedback.Q
% %             fprintf("Ref Q inner: \n");innerloop.Qref
%             fprintf("End inner loop: ===========================\n")
%         end
        
        % =========================
        
        % send vel cmd to motor
        sendJointVel(s, [round(double(Q_cmd_dot_motor'),3), 0], numID);
        %         pause(0.01);
        plotPose.velCmd(:,end+1) = round(double(Q_cmd_dot_motor'),3);
        previousError = [poseError;orError];
    end
    counter= counter+1;
end

%%
% plot ref and current pose
figure(2);
for i = 1:4
    subplot(4,1,i);
    
        plot(Q(i)*ones(size(plotPose.Q(i,:))),'-','color','r','LineWidth',1);hold on
        plot(plotPose.Q(i,:),'-','color','b','LineWidth',1);
        title("Q vs Q_ref ");
    
end
%%
figure(3);
for i = 1:3
    subplot(3,1,i);
    
        plot(double(refPose(i))*ones(size(plotPose.pose(i,:))),'-','color','r','LineWidth',1);hold on
        plot(plotPose.pose(i,:),'-','color','b','LineWidth',1);
        
    
end
title("Pose in time");
%%
figure(4);
for i = 1:3
    subplot(3,1,i);
    plot(plotPose.poseError(i,:),'-','LineWidth',2);
    
end
title("Pose error");
%%
figure(5);
for i = 1:4
    subplot(4,1,i);
    
        plot(plotPose.velCmd(i,:),'-','LineWidth',2);
        
    
end
title("vel cmd ");
%%
figure(5);
for i = 1:4
    subplot(4,1,i);
    
        plot(plotPose.in.Qerror(i,:),'-','LineWidth',2);
        
end
title("Qerror ");
%%

for i = 1:3
    subplot(3,1,i);
    plot(plotPose.orErr(i,:),'-','LineWidth',2);
    
end
title("or error");
%%
% fixAngleFromMotorFB(readMotorFB(s,numID))
substituteVariable(symsVar.J,fixAngleFromMotorFB(readMotorFB(s,numID)))*substituteVariable(symsVar.J_inv,fixAngleFromMotorFB(readMotorFB(s,numID)))
%%
% % Main script
% % init error
% outcome.Joint               = [];
% outcome.pose                = [];
% error.tr.current        = zeros([3,1]);
% error.tr.previous       = zeros([3,1]);
% error.or.current        = zeros([3,1]);
% error.or.previous       = zeros([3,1]);
% motorFB.current         = fixAngleFromMotorFB(readMotorFB(s, 4));
% motorFB.previous        = motorFB.current;
% controller.Xc.current   = zeros([6,1]);
% controller.Xref.previous= zeros([6,1]);
% feedback.jointDisp      = motorFB.current % home position
% feedback.RMtrx          = substituteVariable(symsVar.T_Eto0(1:3,1:3),feedback.jointDisp);
% feedback.pose           = substituteVariable(symsVar.r_E,feedback.jointDisp);
% defaultPose             = feedback.pose;
%
% setControlMode(s, "velocity");
% % loop through each segment (1 -> 3)
% while (true)
%
%     outerLoop = tic;
%
%     % Read joystick input
%     [xJoy, yJoy, eJoy] = readJoy(s);
%     [xJoy, yJoy]                                    % Display joystick input
%     [errorX, errorY] = errorFromJoystick(xJoy,yJoy, current.pose(1), current.pose(2));
%     % update (k-1) before calculate (k)
%     % error
%     error.tr.previous = error.tr.current;
%     error.or.previous = error.or.current;
%     error.outer.previous = [error.tr.current; error.or.current];
%     controller.Xc.previous = controller.Xc.current;
%     % orientation
%     ref.Xe = feedback.RMtrx(:,1);
%     ref.Ye = feedback.RMtrx(:,2);
%     ref.Ze = feedback.RMtrx(:,3);
%     [feedback.Xe, feedback.Ye, feedback.Ze] = GetOrientation(feedback.RMtrx);
%
%     % calculate error (k)
%     error.tr.current = defaultPose - feedback.pose;
% %     error.or.current = getOrientationError(ref,feedback);
%     fprintf("TR ERROR: \n");
%     error.tr.current'
%
%     % --------------------------- controller PI ---------------
%     dt_outerloop                = toc(outerLoop);outerLoop = tic;                               % get sampling time (s)
%     controller.Xref.current     = zeros([6,1]);
%     error.outer.current         = [error.tr.current; error.or.current];
%     controller.Pterm            = controller.Kp * (error.outer.current - error.outer.previous);
% %     controller.Iterm            = controller.Xc.previous - controller.Xref.previous + ...
% %         (controller.Ki*dt_outerloop/2)*(error.outer.current+error.outer.previous);
%     controller.Pterm'
%     controller.Xc.current       = controller.Xref.current + controller.Pterm;
%     fprintf("Controller Xc \n")
%     controller.Xc'
%     controller.Xref.previous    = controller.Xref.current;
%     controller.JMtrx            = substituteVariable(symsVar.J,feedback.jointDisp);
%     controller.invJMtrx         = controller.JMtrx'/(controller.JMtrx*controller.JMtrx');
%     controller.JointVelCmdSyms  = controller.invJMtrx * controller.Xc.current;
%     controller.JointVelCmdNumeric = double(controller.JointVelCmdSyms);
%
%     % use controller.jointVelCmd, send to the motor
%
% %     % --------------------------- Inner loop trigger ------------------------
% %     innerloop.Qref              = InverseKinematicsGeneric(ref.segment.poseRef(:,j));
% %     innerloop_tic               = tic;
% %     innerLoop.feedback.Q        = feedback.jointDisp;
% %     innerLoop.Q_dot             = fixVelMotor(controller.JointVelCmdNumeric);
% %
% %     for k = 1:5
% %         fprintf("Start inner loop: ========================")
% %         innerLoop.error.current = innerloop.Qref' - innerLoop.feedback.Q';
% %         fprintf("Inner error: \n");innerLoop.error
% %         innerLoop.Q_dotCmd      = innerLoop.Q_dot + innerLoop.controller.Kp* innerLoop.error.current;
% %         dt_innerLoop = toc(innerloop_tic);
% %         fprintf("Inner loop sampling time: \n");
% %         dt_innerLoop
% %         fprintf("Inner loop ang vel cmd: \n");
% %         fixVelMotor(innerLoop.Q_dotCmd)
% %         % need to convert Q dot for motor
% %         sendJointVel(s, round(fixVelMotor(innerLoop.Q_dotCmd),2), numID);
% %
% %         % a delay for it to run
% %         pause(dt_innerLoop);
% %
% %         innerLoop.feedback.Q = fixAngleFromMotorFB(readMotorFB(s, 4));
% %         innerLoop.error.prev = innerLoop.error.current;
% %         fprintf("Inner Q fb: \n");innerLoop.feedback.Q
% %         fprintf("Ref Q inner: \n");innerloop.Qref
% %         fprintf("End inner loop: ===========================")
% %     end
%
%     % SEND COMMAND without gripper
%     %         newVel = fixVeltoMotor(controller.JointVelCmdNumeric);
%     %         sendJointVel(s, round(newVel,2), numID);
%
%     % send command with gripper
%     %         sendJointVel(s, [fixVeltoMotor(controller.JointVelCmd),0], numID);
%
%     % get feedback from the motor
%     % take consider of the offset
%     %         [motorFB.current, ~] = readFB(s, numID);
%     %         motorFB.current = motorFB.current(1:4);     % ignore the one on the gripper
%     % joint displacement Q
%     motorFB.previous        = motorFB.current;
%     motorFB.current         = fixAngleFromMotorFB(readMotorFB(s, 4));
%     feedback.jointDisp      = motorFB.current;
%     outcome.Joint           = feedback.jointDisp;
%     fprintf("Sampling time: \n");dt_outerloop
%     % joint velocity Q_dot (rad/s)
%     feedback.jointVel           = (motorFB.current - motorFB.previous)/dt_outerloop;
%     % task space pose feedback r_E, FORWARD KINEMATICS
%     feedback.pose               = substituteVariable(symsVar.r_E,feedback.jointDisp);
%     fprintf("Feedback pose: \n");
%     feedback.pose'
%     fprintf("Ref pose: \n");
%
%     outcome.pose         = double(feedback.pose);
%     feedback.RMtrx              = substituteVariable(symsVar.T_Eto0(1:3,1:3),feedback.jointDisp);
%
%     sendJointVel(s, fixVelMotor(feedback.jointVel), numID);
%     % Trigger the gripper motor
% end

%%
% Close Serial Connection - MUST DO PRIOR TO DISCONNECTING HARDWARE OR CLEARING VARIABLES
fclose(s);


%%
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
orError =   1/2*[cross(fb.Xe,ref.Xe) + cross(fb.Ye,ref.Ye) + cross(fb.Ze,ref.Ze)];
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
% q1Zero = -pi;
q2Zero = -pi/2; % Joint displacement for sensorID = 2 to return to zero position
q3Zero = pi/4; % _____________________________= 3 __________________________
q4Zero = 0;
offset = [q2Zero, q3Zero, q4Zero];

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

function GetBackHome(s, numID, motorQ,gripper, error)
% This is where you will write a sequence to send all of your joints to their home positions.
% For home position, [Q1 Q2 Q3 Q4] = [0, pi/2, -pi/2, -pi/2]
sendJointPos(s,[motorQ+error,gripper], numID);
readMotorFB(s,numID);
end


function [errorX, errorY] = errorFromJoystick(x,y, currentX, currentY)
defaultJoystick = [516,512];
offset = ([x,y]-defaultJoystick(1));
errorX = offset(1)/1023*20;
errorY = offset(2)/1023*20;

if ((errorX + currentX) > 550 ||((errorX + currentX) < 250))
    errorX = 0;
end

if ((errorY + currentY) > 150 ||((errorY + currentY) < -150))
    errorY = 0;
end
end


