clear;
clf;
clc;
axis equal
hold on
pose1{1} = transl(0.25,0,0);
pose2{1} = transl(0.3,0,0);
pose3{1} = transl(0.35,0,0);
pose4{1} = transl(0.6,0,0);
pose5{1} = transl(0.7,0,0);
pose6{1} = transl(0.85,0,0)*trotz(3*pi/2);
pose7{1} = transl(1.1,0,0)*trotz(3*pi/2);
pose8{1} = transl(1.35,0,0)*trotz(3*pi/2);
pose9{1} = transl(1.55,0,0);
pose10{1} = transl(1.85,0,0);
pose11{1} = transl(2.05,0,0);
pose12{1} = transl(2.15,0,0);

RobotSpawn('Pencil',1,pose1)
RobotSpawn('Pen',1,pose2)
RobotSpawn('Ruler',1,pose3)
RobotSpawn('Calculator',1,pose4)
RobotSpawn('Eraser',1,pose5)
RobotSpawn('BookChem',1,pose6)
RobotSpawn('BookMath',1,pose7)
RobotSpawn('BookEng',1,pose8)
RobotSpawn('Compass',1,pose9)
RobotSpawn('Keyboard',1,pose10)
RobotSpawn('Mouse',1,pose11)
RobotSpawn('PS5_Controller',1,pose12)
