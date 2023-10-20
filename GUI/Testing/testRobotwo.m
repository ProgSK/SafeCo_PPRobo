clear;
clf;
clc;
r = Pulse75;
view(3)

% Set the size of the workspace when drawing the robot
workspace = [-1 1 -1 1 0 2];
scale = 0.5;

% Set q values
q = zeros(1,r.model.n);

% Plot the robot
% r.model.plot(q,'workspace',workspace,'scale',scale);
% r.model.plot(q,'workspace',workspace,'scale',scale,'nowrist','nojoints','notiles','noarrow');
r.model.teach();

%% Random robot movement joints (Dance party)
% steps = 50;
% iteration = 10;
% 
% qMatrix = zeros(steps*iteration,6);
% 
% for i = 1:iteration
%     randq1 = randi(rad2deg([r.model.qlim(1,1) r.model.qlim(1,2)]),1,1);
%     randq2 = -abs(randi(rad2deg([r.model.qlim(2,1) r.model.qlim(2,2)]),1,1));
%     randq3 = randi(rad2deg([r.model.qlim(3,1) r.model.qlim(3,2)]),1,1);
%     randq4 = randi(rad2deg([r.model.qlim(4,1) r.model.qlim(4,2)]),1,1);
%     randq5 = randi(rad2deg([r.model.qlim(5,1) r.model.qlim(5,2)]),1,1);
%     randq6 = randi(rad2deg([r.model.qlim(6,1) r.model.qlim(6,2)]),1,1);
%     if i == 1
%         q1 = r.model.getpos();
%         q2 = deg2rad([randq1 randq2 randq3 randq4 randq5 randq6]);
%         qMatrix(i:steps,:) = jtraj(q1,q2,steps);
%     else
%         q1 = q2;
%         q2 = deg2rad([randq1 randq2 randq3 randq4 randq5 randq6]);
%         qMatrix(((i-1)*steps)+1:(i)*steps,:) = jtraj(q1,q2,steps);
%     end
% end
% 
% for j = 1:length(qMatrix)
%     r.model.animate(qMatrix(j,:));
%     drawnow();
% end
