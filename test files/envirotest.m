
envGen()
%% ENVIRONMENT GENERATION

function envGen()

    % % STEP 1: Create/Import the Ground/Floor Texture Map
    %     cornerPoint = 2.5;
    %     surf([-cornerPoint,-cornerPoint ; cornerPoint,cornerPoint] ...
    %         ,[-cornerPoint,cornerPoint ; -cornerPoint,cornerPoint] ...
    %         ,[0.01,0.01 ; 0.01,0.01] ...
    %         ,'CData',imread('3D Models/FloorDesign.jpg') ...
    %         ,'FaceColor','texturemap');
    %     hold on;

    % % STEP 2: Create/Import the Room Model
    %     PlaceObject('3D Models/RoomDesign.ply',[0,0,0]);
    %     PlaceObject('3D Models/BedDesign.ply',[-1,-1.2,0]);

    % STEP 3: Create/Import the Desktop Model/s
        % PlaceObject('3D Models/DeskDesign.ply',[-0.5,1.8,0]);
        % PlaceObject('3D Models/ChairDesign.ply',[-0.5,1.3,0]);
        % PlaceObject('3D Models/MonitorDesign.ply',[0.7,2,0.94]);
        % PlaceObject('3D Models/EstopDesign.ply',[-1.4,1.45,0.85]);
        PlaceObject('3D Models/HolderDesign.ply',[0.2,2,0.94]);
        % PlaceObject('3D Models/BottleDesign.ply',[-1,1.6,0.9]);

    % STEP 4: Create/Import Shelf Model/s
        PlaceObject('3D Models/WShelfDesign1.ply',[-1.22,2.4,1.5]);
        % PlaceObject('3D Models/FShelfDesign.ply',[0,0,0]); If wall shelf isn't good enough

    % STEP 5: Create/Import Human Model
        PlaceObject('3D Models/HumanDesign.ply',[1.5,-1.5,0]);
        
end

