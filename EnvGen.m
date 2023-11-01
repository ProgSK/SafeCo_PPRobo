%% GENERAL CODE

% Global Axis for Models
    axis on;
    workspace = [-4.4 4.4 -4.4 4.4 0 4];
    axis auto
    view(3);

% Global Variables for Modular Position Co-ods
    rOrigin75 = transl([-1.9,1.8,0.945]); % Accounting for Relocation of Robot on Mounting Table
    rOriginDB = transl([-0.1,2,0.945]); % Accounting for Relocation of Robot on Mounting Table

% Called Functions
    envGen() % Function to Generate Environment & all its Safety Features

% Robot Created
    % robo75 = Pulse75();
    robo75 = Pulse75(rOrigin75); % Creating the Pulse75 with specific origin point
    roboDB = DobotMagician(rOriginDB); % Creating the DoBot with specific origin point

% Gripper Created
    gTR = robo75.model.fkine(robo75.model.getpos()).T;
    g1 = Gripper2F85(gTR);
    g2 = Gripper2F85(gTR*trotz(pi));


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
    else
        txt == 'N'
        delete spawnData.mat
    end
end
    

%% OBJECT POSE ALLOCATION
 
% STEP 1: Assigning the Initial Pose Values for the various loads (position of origin before animated movement)
    pencilPose{1} = transl(0.23,2,1.03);
    penPose{1} = transl(0.2,2,1.08);
    rulerPose{1} = transl(0.2,2.03,1.1);
    calcPose{1} = transl(0.05,2,0.94);
    % eraserPose{1} = transl(-0.45,+0.35,+0);
    compassPose{1} = transl(0.155,2,1.071);
    bookChemPose{1} = transl(-0.95,2.255,1.5);
    bookMathPose{1} = transl(-1.05,2.255,1.5);
    bookEngPose{1} = transl(-1.15,2.255,1.5);
    keyboardPose{1} = transl(-1.1,2.4,1.67);
    mousePose{1} = transl(-1.4,2.4,1.67);
    ps5ControllerPose{1} = transl(-1.4,2.4,1.35);

% STEP 2: Assigning the Destination Pose Values for the various loads (position after movement is completed)
    % pencilFinalPose{1} = transl(-0.37,-0.35,+0.2);
    % penFinalPose{1} = transl(-0.75,-0.35,+0);
    % rulerFinalPose{1} = transl(-0.70,-0.35,+0);
    % calcFinalPose{1} = transl(-0.55,-0.35,+0);
    % eraserFinalPose{1} = transl(-0.45,-0.35,+0);
    % compassFinalPose{1} = transl(-0.55,-0.25,+0);
    % bookChemFinalPose{1} = transl(-0.25,-0.3,+0)*l_RotZ;
    % bookMathFinalPose{1} = transl(+0,-0.3,+0)*l_RotZ);
    % bookEngFinalPose{1} = transl(+0.25,-0.3,+0)*l_RotZ;
    % keyboardFinalPose{1} = transl(-0.6,-0,+0);
    % mouseFinalPose{1} = transl(+0.5,-0.25,+0);
    % ps5ControllerFinalPose{1} = transl(+0.7,-0.25,+0);

%% OBJECT SPAWN-IN

if exist(spawnFileName,'file') == 2
    load spawnData.mat % Load pre-existing objects
    % plots the objects (with no colour, PURELY FOR DEBUGGING PURPOSES)
    plot3d(pencilObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(penObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(rulerObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    plot3d(calcObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
    % plot3d(eraserObj.robotModel{1},0,'workspace',workspace,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
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
    % eraserObj = RobotSpawn('Eraser',1,eraserPose);
    chemBookObj = RobotSpawn('BookChem',1,bookChemPose);
    mathBookObj = RobotSpawn('BookMath',1,bookMathPose);
    engBookObj = RobotSpawn('BookEng',1,bookEngPose);
    compassObj = RobotSpawn('Compass',1,compassPose);
    keyboardObj = RobotSpawn('Keyboard',1,keyboardPose);
    mouseObj = RobotSpawn('Mouse',1,mousePose);
    ps5ControllerObj = RobotSpawn('PS5_Controller',1,ps5ControllerPose);
    % save spawnData.mat pencilObj penObj rulerObj calcObj eraserObj chemBookObj mathBookObj engBookObj compassObj keyboardObj mouseObj ps5ControllerObj
    save spawnData.mat pencilObj penObj engBookObj chemBookObj mathBookObj ps5ControllerObj keyboardObj mouseObj rulerObj compassObj calcObj
end

    
%% ENVIRONMENT GENERATION

function envGen()

    % STEP 1: Create/Import the Ground/Floor Texture Map
        cornerPoint = 2.5;
        surf([-cornerPoint,-cornerPoint ; cornerPoint,cornerPoint] ...
            ,[-cornerPoint,cornerPoint ; -cornerPoint,cornerPoint] ...
            ,[0.01,0.01 ; 0.01,0.01] ...
            ,'CData',imread('3D Models/FloorDesign.jpg') ...
            ,'FaceColor','texturemap');
        hold on;

    % STEP 2: Create/Import the Room Model
        PlaceObject('3D Models/RoomDesign.ply',[0,0,0]);
        PlaceObject('3D Models/BedDesign.ply',[-1,-1.2,0]);

    % STEP 3: Create/Import the Desktop Model/s
        PlaceObject('3D Models/DeskDesign.ply',[-0.5,1.8,0]);
        PlaceObject('3D Models/ChairDesign.ply',[-0.5,1.3,0]);
        PlaceObject('3D Models/MonitorDesign.ply',[0.7,2,0.94]);
        PlaceObject('3D Models/EstopDesign.ply',[-1.4,1.45,0.85]);
        PlaceObject('3D Models/HolderDesign.ply',[0.2,2,0.94]);
        PlaceObject('3D Models/BottleDesign.ply',[-1,1.6,0.9]);

    % STEP 4: Create/Import Shelf Model/s
        PlaceObject('3D Models/WShelfDesign1.ply',[-1.22,2.4,1.5]);
        % PlaceObject('3D Models/FShelfDesign.ply',[0,0,0]); If wall shelf isn't good enough

    % STEP 5: Create/Import Human Model
        PlaceObject('3D Models/HumanDesign.ply',[1.5,-1.5,0]);
        
end

