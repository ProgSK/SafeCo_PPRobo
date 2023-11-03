clear;
clf;
clc;

% load environment (EnvGen) script, fills workspace with relevant variables
EnvGen();
r = robo75;
d = roboDB;

% properties from Animation class (used for only for testing purposes/debugging)
self.estop = 0;
self.robotRunning = 0;

% input intermediary and final poses for book
bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
bookEngPoseFinal{1} = transl(-1.1,1.8,1.13) * troty(pi/2);

penPoseIntermediary{1} = penPose{1} * transl(0,0,0.8);
penPoseFinal{1} = transl(-0.1,1.75,0.945) * trotx(pi/2);

rulerPoseIntermediary{1} = rulerPose{1} * transl(0,0,0.8);
rulerPoseFinal{1} = transl(0,1.75,0.945) * trotx(pi/2);

%% Adjust the position of robot and gripper
r.model.animate([-0.5   -1.5708   -1.5708         pi/2         0         0         0])
g1.model.base = g1.model.base.T * trotz(-pi/2);
g2.model.base = g2.model.base.T * trotz(pi/2);
%% Move the Engineering book back and forth using RMRC

% Go from initial position to book pos
InitialQ = r.model.getpos();
animateRMRC(self,r,g1,g2,0,r.model.fkine(InitialQ).T,bookEngPose{1},InitialQ)

% Close the gripper
gripperAnimate(g1,g2,1);

% Grab book from shelf
InitialQ = [-0.5   -1.6982   -45*pi/180    60*pi/180         160*pi/180         0         0];
animateRMRC(self,r,g1,g2,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(self,r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)

% Open the gripper
gripperAnimate(g1,g2,2);

% Make Pulse75 go home
InitialQ = [-0.5   -1.5708   -1.5708         pi/2         0         0         0];
AnimateBricks(self,r,g1,g2,r.model.fkine(InitialQ).T,0,InitialQ);

% Make Dobot pick up pen
InitialQ = [0    0.7854    0.7854    1.5708         0];
AnimateBricks(self,d,0,0,penPose{1},0,InitialQ);

InitialQ = d.model.getpos();
AnimateBricks(self,d,0,0,penPoseIntermediary{1},penObj,InitialQ);

InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
AnimateBricks(self,d,0,0,penPoseFinal{1},penObj,InitialQ);

% Make Dobot pick up ruler
InitialQ = [0    0.7854    0.7854    1.5708         0];
AnimateBricks(self,d,0,0,rulerPose{1},0,InitialQ);

InitialQ = d.model.getpos();
AnimateBricks(self,d,0,0,rulerPoseIntermediary{1},rulerObj,InitialQ);

InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
AnimateBricks(self,d,0,0,rulerPoseFinal{1},rulerObj,InitialQ);

% Make dobot go home
InitialQ = [0 0 0 0 0];
AnimateBricks(self,d,0,0,d.model.fkine(InitialQ).T,0,InitialQ);

