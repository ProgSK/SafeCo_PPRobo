%%
clear;
clf;
clc;

%% Serial Link test of pulse75 DH Parameters
% Create the Pulse75 model
% link(1) = Link([0 0.2325 0 pi/2 0]);
% link(2) = Link([0 0.2 -0.375 0 0]);
% link(3) = Link([0 -0.2 -0.295 0 pi]);
% link(4) = Link([0 0.2 0 pi/2 0]);
% link(5) = Link([0 0.1711 0 -pi/2	0]);
% link(6) = Link([0 0.1226 0 0 0]);

link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
link(2) = Link([0 0.2325 0 pi/2 0]);
link(3) = Link([0 0.135 -0.375 0 0]);
link(4) = Link([0 -0.115 -0.295 0 pi]);
link(5) = Link([0 0.115 0 pi/2 0]);
link(6) = Link([0 0.1711 0 -pi/2	0]);
link(7) = Link([0 0.1226 0 0 0]);

% Incorporate joint limits
link(1).qlim = [0 0.8];
link(2).qlim = [-360 360]*pi/180;
link(3).qlim = [-360 360]*pi/180;
link(4).qlim = [-160 160]*pi/180;
link(5).qlim = [-360 360]*pi/180;
link(6).qlim = [-360 360]*pi/180;
link(7).qlim = [-360 360]*pi/180;

link(3).offset = -pi/2;
link(5).offset = -pi/2;

model = SerialLink(link,'name','Pulse75');

% Set the size of the workspace when drawing the robot
workspace = [-1 1 -1 1 0 2];
scale = 0.5;

% Set q values
q = zeros(1,model.n);

model.base = eye(4)*trotx(pi/2)*troty(pi/2);

% Plot the robot
model.plot(q,'workspace',workspace,'scale',scale);

model.teach();