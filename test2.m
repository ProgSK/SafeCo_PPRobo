clear;
clf;
clc;
pose = cell(1,1);
pose{1} = eye(4);
roobot = RobotSpawn('LinearUR3Link4',1,pose)
handles = findobj('Tag', roobot.robotModel{1}.name);
h = get(handles,'UserData');
% axis auto
% PlaceObject('3D Models/Pencil.PLY',[0,0,0])
% [TRI,PTS,DATA] = plyread('3D Models/Pencil.PLY','tri');
% model.faces = {TRI};
% 
% model.points = {PTS};
% plot3d(model);