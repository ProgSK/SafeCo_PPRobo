clc;
clf;
clear;
r = DobotMagician;
q = r.model.getpos()
T = r.model.fkine(q)
r.model.teach(q)