%% Verify IK by selecting 4 corners of the board

% r_shortest = [110;0;20;1];
% r_shortCorner_1 = [110;65;20;1];
% r_shortCorner_2 = [110;-65;20;1];
% r_farCorner_1 = [240;65;20;1];
% r_farCorner_2 = [240;-65;20;1];
% 
% figure(1);
% Q_shortest = InverseKinematicsGeneric(r_shortest);
% title("Configuration 1: Nearest point");
% disp(Q_shortest);
% 
% figure(2);
% subplot(1,2,1);
% Q_shortCorner_1 = InverseKinematicsGeneric(r_shortCorner_1);
% title("Configuration 2: Nearest left corner");
% disp(Q_shortCorner_1);
% 
% subplot(1,2,2);
% Q_shortCorner_2 = InverseKinematicsGeneric(r_shortCorner_2);
% title("Configuration 3: Nearest right corner");
% disp(Q_shortCorner_2);
% 
% figure(3);
% subplot(1,2,1);
% Q_farCorner_1 = InverseKinematicsGeneric(r_farCorner_1);
% title("Configuration 4: Furthest left corner");
% disp(Q_farCorner_1);
% 
% subplot(1,2,2);
% Q_farCorner_2 = InverseKinematicsGeneric(r_farCorner_2);
% title("Configuration 5: Furthest right corner");
% disp(Q_farCorner_2);

%% Selecting 2 points and verify the IK

r1 = [170;0 ;10;1];
r2 = [110;65 ;20;1];

set1 = InverseKinematicsGeneric(r1);
fprintf("Solution for joint displacement Q1, Q2, Q3, Q4 presenting r1 = [170;0;10;1]:\n");
disp(set1);

set2 = InverseKinematicsGeneric(r2);
fprintf("Solution for joint displacement Q1, Q2, Q3, Q4 presenting r2 = [110;65;20;1]:\n");
disp(set2);
figure(4);
N = length(set1(:,1));
for i  = 1: length(set1(:,1))
subplot(1,2,i);
Visualization(set1(i,1),set1(i,2),set1(i,3),set1(i,4));
tit = "Case " + string(i);
title(tit);
end
sgtitle("Visualization of each Inverse Kinematics solution set for r1");

figure(5);
N = length(set2(:,1));
for i  = 1: length(set2(:,1))
subplot(1,2,i);
Visualization(set2(i,1),set2(i,2),set2(i,3),set2(i,4));
tit = "Case " + string(i);
title(tit);
end
sgtitle("Visualization of each Inverse Kinematics solution set for r2");