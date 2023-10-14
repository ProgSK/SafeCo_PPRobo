clear;
clf;
clc;
axis equal
hold on
%% Initialise poses for manipulatable objects
pencilPose{1} = transl(0.25,0,0);
penPose{1} = transl(0.3,0,0);
rulerPose{1} = transl(0.35,0,0);
calcPose{1} = transl(0.6,0,0);
EraserPose{1} = transl(0.7,0,0);
bookChemPose{1} = transl(0.85,0,0)*trotz(3*pi/2);
bookMathPose{1} = transl(1.1,0,0)*trotz(3*pi/2);
bookEngPose{1} = transl(1.35,0,0)*trotz(3*pi/2);
compassPose{1} = transl(1.55,0,0);
keyboardPose{1} = transl(1.85,0,0);
mousePose{1} = transl(2.05,0,0);
ps5ControllerPose{1} = transl(2.15,0,0);

%% Spawn in robot objects
RobotSpawn('Pencil',1,pencilPose)
RobotSpawn('Pen',1,penPose)
RobotSpawn('Ruler',1,rulerPose)
RobotSpawn('Calculator',1,calcPose)
RobotSpawn('Eraser',1,EraserPose)
RobotSpawn('BookChem',1,bookChemPose)
RobotSpawn('BookMath',1,bookMathPose)
RobotSpawn('BookEng',1,bookEngPose)
RobotSpawn('Compass',1,compassPose)
RobotSpawn('Keyboard',1,keyboardPose)
RobotSpawn('Mouse',1,mousePose)
RobotSpawn('PS5_Controller',1,ps5ControllerPose)
