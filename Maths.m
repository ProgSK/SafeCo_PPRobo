clear;
clf;
clc;

% load environment script, fills workspace with relevant variables
EnvGen();
r = robo75;

% input intermediary and final poses for book
bookMathPoseIntermediary{1} = bookMathPose{1} * transl(0,-0.495,0);
bookMathPoseFinal{1} = transl(-1.1,1.75,1.1) *troty(pi/2);

%% Adjust the position of robot and gripper
r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
g1.model.base = g1.model.base.T * trotz(-pi/2);
g2.model.base = g2.model.base.T * trotz(pi/2);
%% Move the Engineering book back and forth using RMRC

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,0,r.model.fkine(InitialQ).T,bookMathPose{1},InitialQ)

% Close the gripper
gripperAnimate(g1,g2,1);

InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
animateRMRC(r,g1,g2,mathBookObj,bookMathPose{1},bookMathPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,mathBookObj,bookMathPoseIntermediary{1},bookMathPoseFinal{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,mathBookObj,bookMathPoseFinal{1},bookMathPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,mathBookObj,bookMathPoseIntermediary{1},bookMathPose{1},InitialQ)

% Open the gripper 
gripperAnimate(g1,g2,2);