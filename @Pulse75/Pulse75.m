classdef Pulse75 < RobotBaseClass
    %% Pulse75 created by a student

    properties(Access = public)
        plyFileNameStem = 'Pulse75';
    end

    methods
        %% Define robot Function
        function self = Pulse75(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            self.PlotAndColourRobot();
        end

        %% Create the robot model
        function CreateModel(self)
            % Create the Pulse75 model
            link(1) = Link([pi     0       0       pi/2    1]); % PRISMATIC Link
            link(2) = Link([0 0.2725 0 pi/2 0]);
            link(3) = Link([0 0.135 -0.375 0 0]);
            link(4) = Link([0 -0.115 -0.295 0 pi]);
            link(5) = Link([0 0.115 0 pi/2 0]);
            link(6) = Link([0 0.1711 0 -pi/2	0]);
            link(7) = Link([0 0.1226 0 0 0]);

            % Incorporate joint limits
            link(1).qlim = [0 0.8];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-360 360]*pi/180;
            link(4).qlim = [-160 160]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-360 360]*pi/180;
            link(7).qlim = [-360 360]*pi/180;

            self.model = SerialLink(link,'name',self.name);
        end

    end
end