classdef DobotMagician < RobotBaseClass
    %% DobotMagician
    % This class is based on the DobotMagician.
    % URL: https://en.dobot.cn/products/education/magician.html
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access =public)
        plyFileNameStem = 'DobotMagician';
    end

    methods (Access = public)
        %% Constructor
        function self = DobotMagician(baseTr)
            self.CreateModel();
            if nargin < 1
                baseTr = eye(4);
            end
            self.model.base = self.model.base.T * baseTr; % * trotx(pi/2) * troty(pi/2);
            % self.useTool = true;
            % self.toolFilename = 'Online2F85.ply'; % gripper name
            self.PlotAndColourRobot();
        end

        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            link(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            link(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-5),deg2rad(85)]);
            link(4) = Link('d',0,        'a',0.06,      'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-180),deg2rad(180)]);
            link(5) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',pi, 'qlim',[deg2rad(-85),deg2rad(85)]);

            % Incorporate joint limits
            link(1).qlim = [-360 360]*pi/180;
            link(2).qlim = [-90 90]*pi/180;
            link(3).qlim = [-170 170]*pi/180;
            link(4).qlim = [-360 360]*pi/180;
            link(5).qlim = [-360 360]*pi/180;

            self.model = SerialLink(link,'name',self.name);
        end
    end

end