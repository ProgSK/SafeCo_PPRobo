%% Serial Link test of pulse75 DH Parameters
% % Create the Pulse75 model
% link(1) = Link([0 0.2325 0 pi/2 0]);
% link(2) = Link([0 0 -0.375 0 0]);
% link(3) = Link([0 0 -0.295 0 0]);
% link(4) = Link([0 0.1205 0 pi/2 0]);
% link(5) = Link([0 0.1711 0 -pi/2	0]);
% link(6) = Link([0 0.1226 0 0 0]);

% % Create the Pulse75 model
link(1) = Link([0 0.2325 0 pi/2 0]);
link(2) = Link([0 0 -0.375 0 0]);
link(3) = Link([0 0 -0.295 0 0]);
link(4) = Link([0.1205 0 0 pi/2 0]);
link(5) = Link([0.1711 0 0 -pi/2 0]);
link(6) = Link([0.1226 0 0 0 0]);

% Incorporate joint limits
link(1).qlim = [-360 360]*pi/180;
link(2).qlim = [-360 360]*pi/180;
link(3).qlim = [-160 160]*pi/180;
link(4).qlim = [-360 360]*pi/180;
link(5).qlim = [-360 360]*pi/180;
link(6).qlim = [-360 360]*pi/180;

% link(2).offset = -pi/2;
% link(4).offset = -pi/2;

model = SerialLink(link,'name','Pulse75');

% Set the size of the workspace when drawing the robot
workspace = [-1 1 -1 1 -1 1];
scale = 0.5;

% Set q values
q = zeros(1,model.n);

% Plot the robot
model.plot(q,'workspace',workspace,'scale',scale);

model.teach();