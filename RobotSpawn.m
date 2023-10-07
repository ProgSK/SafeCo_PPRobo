classdef RobotSpawn < handle
    %ROBOTBRICKS A class that creates a herd of robot bricks
    %#ok<*TRYNC>

    properties (Constant)
        %> Max height is for plotting of the workspace
        maxHeight = 10;
    end

    properties
        %> Number of robots
        robotCount = 2;

        %> String of robot type
        robotType;

        %> A cell structure of robot models
        robotModel;

        %> A cell structure of robot poses
        robotPoses;

        %> Dimensions of the workspace
        workspaceDimensions;
    end

    methods
        %% ...structors
        function self = RobotSpawn(robotType,robotCount,robotPoses)
            if 0 < nargin
                self.robotCount = robotCount;
                self.robotPoses = robotPoses;
                self.robotType = robotType;
            end
            % Initialise the log file
            % L = log4matlab('logtest.log');

            self.workspaceDimensions = [-3, 4, -2, 2.5, 0, 2.5];

            % Create the required number of bricks
            for i = 1:self.robotCount
                [self.robotModel{i},C] = self.GetRobotModel(self.robotType,i);
                self.robotModel{i}.base = self.robotPoses{i}; % robot 1 is given robot pose 1

                % Plot 3D model
                plot3d(self.robotModel{i},0,'workspace',self.workspaceDimensions,'view',[30,30],'delay',0,'noarrow','nowrist','notiles');
                % Hold on after the first plot (if already on there's no difference)
                if i == 1
                    hold on;
                end
                handles = findobj('Tag', self.robotModel{i}.name);
                h = get(handles,'UserData');
                h.link(i).Children.Faces = self.robotModel{i}.faces{1, 2};
                h.link(i).Children.Vertices = self.robotModel{i}.points{1, 2};
                h.link(i).Children.FaceVertexCData = C;
                h.link(i).Children.FaceColor = 'interp';
                % Store values in log file
                % L.mlog = {L.DEBUG,'RobotSpawn',['h.link(i).Children.Vertices: ',num2str(h.link(i).Children.Vertices)]};
                % L.mlog = {L.DEBUG,'RobotSpawn',['h.link(i).Children.Faces: ',num2str(h.link(i).Children.Faces)]};
                % L.mlog = {L.DEBUG,'RobotSpawn',['h.link(i).Children.FaceVertexCData: ',num2str(h.link(i).Children.FaceVertexCData)]};
                drawnow();
            end

            axis equal
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
        end


        function delete(self)
            for index = 1:self.robotCount
                handles = findobj('Tag', self.robotModel{index}.name);
                h = get(handles,'UserData');
                try delete(h.robot); end
                try delete(h.wrist); end
                try delete(h.link); end
                try delete(h); end
                try delete(handles); end
            end
        end
    end

    methods (Static)
        %% GetBrickModel
        function [model,vertexColours] = GetRobotModel(name,count)
            if nargin < 2
                name = 'Pencil';
            end
            [faceData,vertexData,colorData] = plyread(['3D Models/',name,'.ply'],'tri');
            % VertCount = size(vertexData,1);
            % midPoint = sum(vertexData)/VertCount;
            % Verts = vertexData - repmat(midPoint,VertCount,1);
            % Pose = eye(4);
            vertexColours = [colorData.vertex.red ...
                , colorData.vertex.green ...
                , colorData.vertex.blue]/255;
            % Mesh_h = trisurf(faceData,Verts(:,1),Verts(:,2),Verts(:,3),'FaceVertexCData',...
            %     vertexColours,'EdgeColor','interp');
            link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',[name,num2str(count)]);

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