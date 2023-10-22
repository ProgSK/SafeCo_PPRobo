clear;
clf;
clc;
%% Load pre-existing mat files
% Specify the name of the specific .mat file you're looking for (purely for debugging)
spawnFileName = 'spawnData.mat';
if exist(spawnFileName,'file') == 2
    % The file does exist
    disp(['The file ', spawnFileName, ' was found in the current directory.']);
    prompt = "Do you want to keep this file? Y/N [Y]: ";
    txt = input(prompt,"s");
    if isempty(txt)
        txt = 'Y';
    end
    if txt == 'N'
        delete spawnData.mat
    end
end

%% Initialise variables and axis
Pulse75TR(1) = 0;
Pulse75TR(2) = 0;
Pulse75TR(3) = 0;
steps = 50; %interpolation steps for animation

view(-45,45)
hold on
axis equal
workspace = [-2.5, 3.5, -2, 2, 0, 1.5];
axis(workspace);
if isempty(findobj(get(gca,'Children'),'Type','Light'))
    camlight
end

%% Initialise poses for manipulatable objects
pencilPose{1} = transl(Pulse75TR(1)-0.57,Pulse75TR(2)+0.2,Pulse75TR(3)+0.2);
penPose{1} = transl(Pulse75TR(1)-0.54,Pulse75TR(2)+0.2,Pulse75TR(3)+0);
rulerPose{1} = transl(Pulse75TR(1)-0.70,Pulse75TR(2)+0.35,Pulse75TR(3)+0);
calcPose{1} = transl(Pulse75TR(1)-0.55,Pulse75TR(2)+0.35,Pulse75TR(3)+0);
eraserPose{1} = transl(Pulse75TR(1)-0.45,Pulse75TR(2)+0.35,Pulse75TR(3)+0);
compassPose{1} = transl(Pulse75TR(1)-0.55,Pulse75TR(2)+0.25,Pulse75TR(3)+0);
bookChemPose{1} = transl(Pulse75TR(1)-0.25,Pulse75TR(2)+0.3,Pulse75TR(3)+0)*trotz(3*pi/2);
bookMathPose{1} = transl(Pulse75TR(1)+0,Pulse75TR(2)+0.3,Pulse75TR(3)+0)*trotz(3*pi/2);
bookEngPose{1} = transl(Pulse75TR(1)+0.25,Pulse75TR(2)+0.3,Pulse75TR(3)+0)*trotz(3*pi/2);
keyboardPose{1} = transl(Pulse75TR(1)+0.6,Pulse75TR(2)+0,Pulse75TR(3)+0);
mousePose{1} = transl(Pulse75TR(1)+0.5,Pulse75TR(2)+0.25,Pulse75TR(3)+0);
ps5ControllerPose{1} = transl(Pulse75TR(1)+0.7,Pulse75TR(2)+0.25,Pulse75TR(3)+0);

pencilFinalPose{1} = transl(Pulse75TR(1)-0.37,Pulse75TR(2)-0.35,Pulse75TR(3)+0.2);
penFinalPose{1} = transl(Pulse75TR(1)-0.75,Pulse75TR(2)-0.35,Pulse75TR(3)+0);
rulerFinalPose{1} = transl(Pulse75TR(1)-0.70,Pulse75TR(2)-0.35,Pulse75TR(3)+0);
calcFinalPose{1} = transl(Pulse75TR(1)-0.55,Pulse75TR(2)-0.35,Pulse75TR(3)+0);
eraserFinalPose{1} = transl(Pulse75TR(1)-0.45,Pulse75TR(2)-0.35,Pulse75TR(3)+0);
compassFinalPose{1} = transl(Pulse75TR(1)-0.55,Pulse75TR(2)-0.25,Pulse75TR(3)+0);
bookChemFinalPose{1} = transl(Pulse75TR(1)-0.25,Pulse75TR(2)-0.3,Pulse75TR(3)+0)*trotz(3*pi/2);
bookMathFinalPose{1} = transl(Pulse75TR(1)+0,Pulse75TR(2)-0.3,Pulse75TR(3)+0)*trotz(3*pi/2);
bookEngFinalPose{1} = transl(Pulse75TR(1)+0.25,Pulse75TR(2)-0.3,Pulse75TR(3)+0)*trotz(3*pi/2);
keyboardFinalPose{1} = transl(Pulse75TR(1)-0.6,Pulse75TR(2)-0,Pulse75TR(3)+0);
mouseFinalPose{1} = transl(Pulse75TR(1)+0.5,Pulse75TR(2)-0.25,Pulse75TR(3)+0);
ps5ControllerFinalPose{1} = transl(Pulse75TR(1)+0.7,Pulse75TR(2)-0.25,Pulse75TR(3)+0);

%% Spawn in robot objects
% First check to see if there is existing mat file
if exist(spawnFileName,'file') == 2
    load spawnData.mat % Load pre-existing objects
    % plots the objects (with no colour, PURELY FOR DEBUGGING PURPOSES)
    plot3d(pencilObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(penObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(rulerObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(calcObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(eraserObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(chemBookObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(mathBookObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(engBookObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(compassObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(keyboardObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(mouseObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(ps5ControllerObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
else
    pencilObj = RobotSpawn('Pencil',1,pencilPose);
    penObj = RobotSpawn('Pen',1,penPose);
    rulerObj = RobotSpawn('Ruler',1,rulerPose);
    calcObj = RobotSpawn('Calculator',1,calcPose);
    eraserObj = RobotSpawn('Eraser',1,eraserPose);
    chemBookObj = RobotSpawn('BookChem',1,bookChemPose);
    mathBookObj = RobotSpawn('BookMath',1,bookMathPose);
    engBookObj = RobotSpawn('BookEng',1,bookEngPose);
    compassObj = RobotSpawn('Compass',1,compassPose);
    keyboardObj = RobotSpawn('Keyboard',1,keyboardPose);
    mouseObj = RobotSpawn('Mouse',1,mousePose);
    ps5ControllerObj = RobotSpawn('PS5_Controller',1,ps5ControllerPose);
    save spawnData.mat pencilObj penObj rulerObj calcObj eraserObj chemBookObj mathBookObj engBookObj compassObj keyboardObj mouseObj ps5ControllerObj
end

%% Spawn in custom robot
basePulse75 = transl(Pulse75TR);
r = Pulse75(basePulse75);

%% Animate using RMRC

% pencilInitialQ = [0    0.8836   -0.8378    1.7453   -2.4544   -1.5217         0]; % From teach pendant manual movement (eyeballing)
pencilInitialQ = zeros(1,r.model.n);
animateRMRC(r,pencilObj,pencilPose{1},pencilFinalPose{1},pencilInitialQ)

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


%% Inverse kinematics function
function qMatrix = RobotIK(robot, Pose)
objNo = numel(Pose);
qMatrix = cell(1,objNo);
% tr = zeros(4,4,robot.model.n+1);
for i = 1:objNo
    endEffectorPose = Pose{i}; %* transl(0, 0, -0.10);
    qMatrix{i} = robot.model.ikcon(endEffectorPose);

    % % 2.4: Get the transform of every joint (i.e. start and end of every link)
    % tr(:,:,1) = robot.model.base;
    % L = robot.model.links;
    % for j = 1 : robot.model.n
    %     tr(:,:,j+1) = tr(:,:,j) * trotz(qMatrix{i}(1,j)+L(j).offset) * transl(0,0,L(j).d) * transl(L(j).a,0,0) * trotx(L(j).alpha);
    % end
    % if tr(3,4,2) < 0.3
    %     qMatrix{i}(1,2) = -qMatrix{i}(1,2);
    % end
    
end
end

%% Animate ikine
function AnimateBricks(robot,qMatrixInitial,qMatrixFinal,obj,steps)

% Set up variables
noPose = numel(qMatrixInitial);

% Animation for loop for moving each initial and final poses for each brick
for i = 1:noPose

    % Animate the robot going to the initial brick (i) position
    q1 = robot.model.getpos();
    q1 = jtraj(q1,qMatrixInitial{i},steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal

    for j = 1:length(q1)
        robot.model.animate(q1(j,:));
        drawnow();
    end

    % Display current progress and difference between initial brick pose and initial calculated pose
    q1 = robot.model.getpos();
    pause(0.25);
    % Animate the robot going to the final brick (i) position
    q2 = jtraj(q1,qMatrixFinal{i},steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal
    for k = 1:length(q2)
        robot.model.animate(q2(k,:));
        endEffectorPose = robot.model.fkine(robot.model.getpos());

        % While the robot is moving, animate the brick movement
        obj.robotModel{i}.base = endEffectorPose.T; %* transl(0,0,0.10);
        obj.robotModel{i}.animate(0);
        drawnow();
    end
end
end