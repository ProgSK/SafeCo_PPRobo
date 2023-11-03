%% Initiate Environment
% NOTE: change axis in Environmen.m line 14 to change global view axis

%initMsg = msgbox("Designing the environemnt and locating the robots, please wait. This prompt will self-destruct.");

EnvGen();
%delete(initMsg)

%% Initiate Robot(s)
%robot = Pulse75(transl(-1.4, 1.3, 0.94)); % initalise the robot location'
%robot = Pulse75(); % initalise the robot location
%self.robot = robot; %store robot in the class
robo75.advancedTeach; %opens advanced teach
roboDB.expertTeach; %opens advanced teach

%robot = LinearUR3
%robot.expertTeach

% %% Pumpa560
% robot = UR3
% %q = zeros(1,robot.model.n);
% hold on
% robot.model.teach()
% 
