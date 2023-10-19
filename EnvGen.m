%% GENERAL CODE

% Global Axis for Models
    axis on;
    axis ([-4.4 4.4 -4.4 4.4 0 4]);
    axis auto
    view(3);

% % Global Variables for Modular Position Co-ods
    % coodArray = [0,0,0]; % Modular Variable Values (According to Pose Values Assigned during Assessment)
    % rOrigin = transl([coodArray(1),coodArray(2),coodArray(3)+0.5]); % Accounting for Relocation of Robot on Mounting Table

% Called Functions
    envGen() % Function to Generate Environment & all its Safety Features
    % envGen(coodArray) % For modular co-ordinate array

% Robot Created
    robo75 = Pulse75(); % Creating the Robot
    % robo75 = Pulse75(rOrigin); % Creating the Robot with specific origin point


%% OBJECT POSE ALLOCATION
% 
% bRotationX = trotx(pi); 
% bRotationZ = trotz(pi/2);
% 
% % STEP 1: Assigning the Initial Pose Values for the various loads (position of origin before animated movement)
%     bInitpose = cell(1,9);
%     bInitpose{1} = transl(coodArray(1)-0.1,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{4} = transl(coodArray(1)-0.2,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{7} = transl(coodArray(1)-0.3,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{2} = transl(coodArray(1)-0.4,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{5} = transl(coodArray(1)-0.5,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{8} = transl(coodArray(1)-0.6,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{3} = transl(coodArray(1)-0.7,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{6} = transl(coodArray(1)-0.8,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
%     bInitpose{9} = transl(coodArray(1)-0.9,coodArray(2)+0.5,coodArray(3)+0.535) * bRotationX;
% 
% % STEP 2: Assigning the Destination Pose Values for the various loads (position after movement is completed)
%     bDestpose = cell(1,9);
%     bDestpose{1} = transl(coodArray(1)-0.3,coodArray(2)-0.45,coodArray(3)+0.535) * bRotationX * bRotationZ;
%     bDestpose{4} = transl(coodArray(1)-0.435,coodArray(2)-0.45,coodArray(3)+0.535) * bRotationX * bRotationZ;
%     bDestpose{7} = transl(coodArray(1)-0.570,coodArray(2)-0.45,coodArray(3)+0.535) * bRotationX * bRotationZ;
%     bDestpose{2} = transl(coodArray(1)-0.3,coodArray(2)-0.45,coodArray(3)+0.57) * bRotationX * bRotationZ;
%     bDestpose{5} = transl(coodArray(1)-0.435,coodArray(2)-0.45,coodArray(3)+0.57) * bRotationX * bRotationZ;
%     bDestpose{8} = transl(coodArray(1)-0.570,coodArray(2)-0.45,coodArray(3)+0.57) * bRotationX * bRotationZ;
%     bDestpose{3} = transl(coodArray(1)-0.3,coodArray(2)-0.45,coodArray(3)+0.605) * bRotationX * bRotationZ;
%     bDestpose{6} = transl(coodArray(1)-0.435,coodArray(2)-0.45,coodArray(3)+0.605) * bRotationX * bRotationZ;
%     bDestpose{9} = transl(coodArray(1)-0.570,coodArray(2)-0.45,coodArray(3)+0.605) * bRotationX * bRotationZ;

    
%% ENVIRONMENT GENERATION

function envGen()

    % STEP 1: Create/Import the Ground/Floor Texture Map
        cornerPoint = 4.5;
        surf([-cornerPoint,-cornerPoint ; cornerPoint,cornerPoint] ...
            ,[-cornerPoint,cornerPoint ; -cornerPoint,cornerPoint] ...
            ,[0.01,0.01 ; 0.01,0.01] ...
            ,'CData',imread('3D Models/FloorDesign.jpg') ...
            ,'FaceColor','texturemap');
        hold on;

        % For consideration of modular co-ordinate array
        % cornerPoint = 4.5;
        % surf([-cornerPoint+coodArray(1),-cornerPoint+coodArray(1) ; cornerPoint+coodArray(1),cornerPoint+coodArray(1)] ...
        %     ,[-cornerPoint+coodArray(2),cornerPoint+coodArray(2) ; -cornerPoint+coodArray(2),cornerPoint+coodArray(2)] ...
        %     ,[0.01+coodArray(3),0.01+coodArray(3) ; 0.01+coodArray(3),0.01+coodArray(3)] ...
        %     ,'CData',imread('3D Models/FloorDesign.jpg') ...
        %     ,'FaceColor','texturemap');
        % hold on;

    % STEP 2: Create/Import the Room Model
        PlaceObject('3D Models/RoomDesign.ply',[0,0,0]);
        PlaceObject('3D Models/BedDesign.ply',[-2,-2.55,0]);

    % STEP 3: Create/Import the Desktop Model/s
        PlaceObject('3D Models/DeskDesign.ply',[-0.75,2.75,0]);
        PlaceObject('3D Models/ChairDesign.ply',[-0.5,0.5,0]);
        PlaceObject('3D Models/MonitorDesign.ply',[1,3,1.87]);
        PlaceObject('3D Models/EstopDesign.ply',[-2.1,2,1.8]);

    % STEP 4: Create/Import Shelf Model/s
        % PlaceObject('3D Models/WShelfDesign.ply',[0,0,0]);

        % PlaceObject('3D Models/FShelfDesign.ply',[0,0,0]); If wall shelf isn't good enough

    % STEP 5: Create/Import Human Model
        PlaceObject('3D Models/HumanDesign.ply',[3,-3,0]);

end

