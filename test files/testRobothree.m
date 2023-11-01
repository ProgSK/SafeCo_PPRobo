clear;
clf;
clc;
r = DobotMagician;
view(3)

% Set the size of the workspace when drawing the robot
workspace = [-1 1 -1 1 0 2];
scale = 0.5;

% Set q values
q = zeros(1,r.model.n);

% Plot the robot
% r.model.plot(q,'workspace',workspace,'scale',scale);
r.model.plot(q,'workspace',workspace,'scale',scale,'nowrist','nojoints','notiles','noarrow');
r.model.teach();