clear;
clf;
clc;
axis equal
hold on
pose1{1} = transl(0.25,0,0);
pose1{2} = transl(0.3,0,0);


pose2{1} = transl(0.5,0,0);
pose3{1} = transl(0.75,0,0);
pose4{1} = transl(1,0,0);
pose5{1} = transl(1.25,0,0);
pose6{1} = transl(1.5,0,0);
pose7{1} = transl(1.75,0,0);
pose8{1} = transl(2,0,0);

RobotSpawn('Pencil',2,pose1)
RobotSpawn('Pen',1,pose2)
RobotSpawn('Ruler',1,pose3)
RobotSpawn('Calculator',1,pose4)
RobotSpawn('ChemBook',1,pose5)
RobotSpawn('Eraser',1,pose6)
RobotSpawn('MathBook',1,pose7)
