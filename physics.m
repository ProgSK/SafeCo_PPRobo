clear;
clf;
clc;

% load environment script, fills workspace with relevant variables
EnvGen();
r = robo75();

% input intermediary and final poses for book
bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
bookEngPoseFinal{1} = transl(-1.1,1.75,1.1) *troty(pi/2);

% engBookInitialQ = zeros(1,r.model.n)

%% Adjust the position of robot and gripper
r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
g1.model.base = g1.model.base.T * trotz(-pi/2);
g2.model.base = g2.model.base.T * trotz(pi/2);
%% Move the Engineering book back and forth using RMRC

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,0,r.model.fkine(InitialQ).T,bookEngPose{1},InitialQ)
% Close the gripper
gripperAnimate(g1,g2,1);

InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
animateRMRC(r,g1,g2,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)
% AnimateBricks(robo75,g1,g2,bookEngPoseIntermediary{1},bookEngPoseFinal{1},engBookObj,0);

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,engBookObj,bookEngPoseFinal{1},bookEngPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPose{1},InitialQ)

% Open the gripper 
gripperAnimate(g1,g2,2);

%% Find q values of all object poses
% qMatrixPencilInitial = RobotIK(r,pencilPose);
% qMatrixPencilFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixPenInitial = RobotIK(r,penPose);
% qMatrixPenFinal = RobotIK(r,penFinalPose);

% qMatrixPencilInitial = RobotIK(r,pencilPose);
% qMatrixPencilFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixRulerInitial = RobotIK(r,pencilPose);
% qMatrixRulerFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixCalcInitial = RobotIK(r,pencilPose);
% qMatrixCalcFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixEraserInitial = RobotIK(r,pencilPose);
% qMatrixEraserFinal = RobotIK(r,pencilFinalPose);

%% Interpolate and animate the joint values
% AnimateBricks(r,qMatrixPencilInitial,qMatrixPencilFinal,pencilObj,steps);
% AnimateBricks(r,qMatrixPenInitial,qMatrixPenFinal,penObj,steps);

