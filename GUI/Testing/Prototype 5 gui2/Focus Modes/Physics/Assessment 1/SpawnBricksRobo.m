classdef SpawnBricksRobo < handle
    %ROBOTCOWS A class that creates a herd of robot
    %   The cows can be moved around randomly. It is then possible to query
    %   the current location (base) of the cows.

    %#ok<*TRYNC>
    

    properties
        %> Number of cows
        cowCount = 2;

        %> A cell structure of \c cowCount cow models
        cowModel;
      
        %> Dimensions of the workspace in regard to the padoc size
        workspaceDimensions;
    end

    methods
        %% Turning bricks into moveable robots
        function self = SpawnBricksRobo(cowCount,posCow)
            %Might need to delete
            if 0 < nargin
                self.cowCount = cowCount;
            end
            self.workspaceDimensions = [-4, 4, -4, 4, 0, 4];

            % Create the required number of cows
            for i = 1:self.cowCount
                self.cowModel{i} = self.GetCowModel(['cow',num2str(i)]);

                % Identify locations for the amount of bricks
                basePose = posCow{i};
                self.cowModel{i}.base = basePose; %brick 1 is given brick pose 1

                % Plot 3D model
                plot3d(self.cowModel{i},0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1
                    hold on;
                end
            end
            axis equal
        end
    end

    methods(Static)
        %% GetCowModel from RVC tools but instead of moo moo it gets brick
        function model = GetCowModel(name)
            if nargin < 1
                name = 'Cow';
            end
            [faceData,vertexData] = plyread('HalfSizedRedGreenBrick.ply','tri');

            % This is the og link1 parameters but we dont want that for our brick
            % link1 = Link('alpha',pi/2,'a',0,'d',0.3,'offset',0);

            link1 = Link('alpha',0,'a',0,'d',0,'offset',0);

            model = SerialLink(link1,'name',name);

            % Changing order of cell array from {faceData, []} to
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end
    end
end


