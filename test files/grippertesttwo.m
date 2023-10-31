clear;
clf;
clc;

% robo75 = Pulse75();
% 
% gTR = robo75.model.fkine(robo75.model.getpos()).T;
% g1 = Gripper2F85(gTR);
% g2 = Gripper2F85(gTR*trotz(pi));

%%
g1 = Gripper2F85();
g2 = Gripper2F85(trotz(pi));

gripperAnimate(g1,g2,1);
