clc;
clf;
clear;
r = DobotMagician;
q = r.model.getpos()
T = r.model.fkine(q)
r.model.teach(q)

%%testing first commit using git command git commit & git push