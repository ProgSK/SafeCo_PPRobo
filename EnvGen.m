classdef EnvGen < handle
    %EnvGen A class to generate the environment within one script to be called upon by larger script
    %#ok<*TRYNC>


    properties (Constant)
        % Global Variables for Modular Position Co-ods
        coodArray = [0,0,0]; % Modular Variable Values
    end

    methods (Static)
        %% ENVIRONMENT GENERATION

        function self = EnvGen(coodArray)

            % STEP 1: Create/Import the Ground/Floor Texture Map
            cornerPoint = 4.5;
            surf([-cornerPoint+coodArray(1),-cornerPoint+coodArray(1) ; cornerPoint+coodArray(1),cornerPoint+coodArray(1)] ...
                ,[-cornerPoint+coodArray(2),cornerPoint+coodArray(2) ; -cornerPoint+coodArray(2),cornerPoint+coodArray(2)] ...
                ,[0.01+coodArray(3),0.01+coodArray(3) ; 0.01+coodArray(3),0.01+coodArray(3)] ...
                ,'CData',imread('MasonaryFloor.jpg') ...
                ,'FaceColor','texturemap');
            hold on;

            % STEP 2: Create/Import the Room Model
            PlaceObject('3D Models/WallDesign.ply',[coodArray(1),coodArray(2),coodArray(3)]);

            % STEP 3: Create/Import the Desktop Model
            PlaceObject('3D Models/DeskDesign.ply',[coodArray(1)-2.1,coodArray(2)-3.7,coodArray(3)]);
            PlaceObject('3D Models/ChairDesign.ply',[coodArray(1)-2.1,coodArray(2)-3.7,coodArray(3)]);
            PlaceObject('3D Models/MonitorDesign.ply',[coodArray(1)-2.1,coodArray(2)-3.7,coodArray(3)]);

            % STEP 4: Create/Import Shelf Model/s
            PlaceObject('3D Models/WShelfDesign.ply',[coodArray(1),coodArray(2),coodArray(3)]);

            % PlaceObject('3D Models/FShelfDesign.ply',[coodArray(1),coodArray(2),coodArray(3)]);

            % STEP 5: Create/Import Human Model
            PlaceObject('3D Models/HumanDesign.ply',[coodArray(1),coodArray(2),coodArray(3)]);

        end

    end
end