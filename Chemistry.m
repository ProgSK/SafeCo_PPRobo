clear;
clf;
clc;

% load environment script, fills workspace with relevant variables
EnvGen();
r = robo75;

% input intermediary and final poses for book
bookChemPoseIntermediary{1} = bookChemPose{1} * transl(0,-0.495,0);
bookChemPoseFinal{1} = transl(-1.1,1.75,1.1) *troty(pi/2);

%% Adjust the position of robot and gripper
r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
g1.model.base = g1.model.base.T * trotz(-pi/2);
g2.model.base = g2.model.base.T * trotz(pi/2);
%% Move the Engineering book back and forth using RMRC

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,0,r.model.fkine(InitialQ).T,bookMathPose{1},InitialQ)

% Close the gripper
gripperAnimate(g1,g2,1);

% Move the Chemistry book back and forth using RMRC
InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,chemBookObj,bookChemPose{1},bookChemPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,chemBookObj,bookChemPoseIntermediary{1},bookChemPoseFinal{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,chemBookObj,bookChemPoseFinal{1},bookChemPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,chemBookObj,bookChemPoseIntermediary{1},bookChemPose{1},InitialQ)

% Open the gripper 
gripperAnimate(g1,g2,2);