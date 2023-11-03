%ANIMATION This class controls the location and movement of the robots and
%   bricks within the animated wall building

classdef Animation < handle
    properties
        %GUI variables
        estop; % estopbutton control
        startRobot; % stops animation from starting till start is pressed
        robotRunning; % stop animation from continuing after G till continue is pressed
        orderReady; % stores which order is ready. If 0 no order is ready. Values can be 0-3
        %
        % %Setup Robots Variables
        robot;      %IRB120
        % robotBase;  %IRB120 Base Location
        %
        % %Setup Cup Variables
        % cups;
        % cupEndLocations;
        % cupStartLocations;
        %
        % %Setup Person Variables
        % person;
        % personStartLocation;
        % personEndLocation;
        %
        % %Animation Variables
        % qMatrix;
        % orderNo;
    end

    methods
        function self = Animation()

            disp('setting up robot...');
            %% Animation setup
            % EnvGen();
            % r = robo75;

            % % robot Location
            % % cup start and end locations
            % % ->getStartingPositions
            % [cupStartLocations, cupEndLocations,personStartLocation, ...
            %     personEndLocation, robotBase] = self.getStartingPositions();

            % %store the variables in the object
            % self.robotBase = robotBase; % not used
            % self.cupStartLocations = cupStartLocations; %cup starting locations
            % self.cupEndLocations = cupEndLocations;
            % self.personStartLocation = personStartLocation;
            % self.personEndLocation = personEndLocation;

            % % input intermediary and final poses for book
            % bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
            % bookEngPoseFinal{1} = transl(-1.1,1.75,1.1) *troty(pi/2)
            %
            % %% Adjust the position of robot and gripper
            % r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
            % g1.model.base = g1.model.base.T * trotz(-pi/2);
            % g2.model.base = g2.model.base.T * trotz(pi/2);

            % %initiate the robot
            robot = Pulse75(); % initalise the robot location
            self.robot = robot; %store robot in the class
            % robot.advancedTeach; %opens advanced teach

            % %make an array of cup
            % for i = 1:3
            %     self.cups{i} = Thing("cup5",self.cupStartLocations{i});
            % end
            %
            % %make an array of person
            % for i = 1:3
            %     self.person{i} = Thing("person7",self.personStartLocation{i});
            % end
            %
            % %get the moves for each robot and whether the brick is picked up
            % [cup, cupMoving, person, personMoving, cupTR] = getMoves(self);
            %
            % %convert the transforms to joint positions for each move per robot
            % qMatrix = self.transformMoves(self.robot, cupTR);
            % self.qMatrix = qMatrix;
            %
            % % reset the view range
            % axis([-3 3 -3 3 0 2.5]);

            %--------GUI properties
            self.estop = 0;
            self.robotRunning = 1;
            self.startRobot = 0;
            self.orderReady = 0;

            self.getSimulationGUI; % starts the GUI for robot control
            %------------------------
            disp('WELCOME!');


            %-----------Loop for cups
            while 1
                switch self.orderReady
                    case 0
                        pause(1);
                    case 1
                        %% call in physics function
                        self.physics(self.robot);
                        self.orderReady = 0;
                        self.startRobot = 0;
                        self.robotRunning = 0;
                        disp("Physics book, pen and ruler ready!");
                    case 2
                        %% call in chemistry function
                        self.chemistry(self.robot);
                        self.orderReady = 0;
                        self.startRobot = 0;
                        self.robotRunning = 0;
                        disp("Chemistry book, pen and pencil ready!");
                    case 3
                        %% call in chemistry function
                        self.maths(self.robot);
                        self.orderReady = 0;
                        self.startRobot = 0;
                        self.robotRunning = 0;
                        disp("Brought out Maths book, a pen and a compass!");
                end
            end
            %------------------------------------------------

        end


        %% Physics
        function physics(self, robot)

            while self.startRobot == 0
                pause(1);
            end
            if self.startRobot == 1
                % load environment (EnvGen) script, fills workspace with relevant variables
                EnvGen();
                r = robo75;
                d = roboDB;

                % properties from Animation class (used for only for testing purposes/debugging)
                self.estop = 0;
                self.robotRunning = 0;

                % input intermediary and final poses for book
                bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
                bookEngPoseFinal{1} = transl(-1.1,1.8,1.13) * troty(pi/2);

                penPoseIntermediary{1} = penPose{1} * transl(0,0,0.8);
                penPoseFinal{1} = transl(-0.1,1.75,0.945) * trotx(pi/2);

                rulerPoseIntermediary{1} = rulerPose{1} * transl(0,0,0.8);
                rulerPoseFinal{1} = transl(0,1.75,0.945) * trotx(pi/2);

                %% Adjust the position of robot and gripper
                r.model.animate([-0.5   -1.5708   -1.5708         pi/2         0         0         0])
                g1.model.base = g1.model.base.T * trotz(-pi/2);
                g2.model.base = g2.model.base.T * trotz(pi/2);
                %% Move the Engineering book back and forth using RMRC

                % Go from initial position to book pos
                InitialQ = r.model.getpos();
                animateRMRC(self,r,g1,g2,0,r.model.fkine(InitialQ).T,bookEngPose{1},InitialQ)

                % Close the gripper
                gripperAnimate(g1,g2,1);

                % Grab book from shelf
                InitialQ = [-0.5   -1.6982   -45*pi/180    60*pi/180         160*pi/180         0         0];
                animateRMRC(self,r,g1,g2,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)

                InitialQ = r.model.getpos();
                animateRMRC(self,r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)

                % Open the gripper
                gripperAnimate(g1,g2,2);

                % Make Pulse75 go home
                InitialQ = [-0.5   -1.5708   -1.5708         pi/2         0         0         0];
                AnimateBricks(self,r,g1,g2,r.model.fkine(InitialQ).T,0,InitialQ);

                % Make Dobot pick up pen
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,penPose{1},0,InitialQ);

                InitialQ = d.model.getpos();
                AnimateBricks(self,d,0,0,penPoseIntermediary{1},penObj,InitialQ);

                InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
                AnimateBricks(self,d,0,0,penPoseFinal{1},penObj,InitialQ);

                % Make Dobot pick up ruler
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,rulerPose{1},0,InitialQ);

                InitialQ = d.model.getpos();
                AnimateBricks(self,d,0,0,rulerPoseIntermediary{1},rulerObj,InitialQ);

                InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
                AnimateBricks(self,d,0,0,rulerPoseFinal{1},rulerObj,InitialQ);

                % Make dobot go home
                InitialQ = [0 0 0 0 0];
                AnimateBricks(self,d,0,0,d.model.fkine(InitialQ).T,0,InitialQ);
            end

        end

        %% Chemistry mode
        function chemistry(self, robot)

            while self.startRobot == 0
                pause(1);
            end
            if self.startRobot == 1
                % load environment script, fills workspace with relevant variables
                EnvGen();
                r = robo75;
                d = roboDB;

                % properties from Animation class (used for only for testing purposes/debugging)
                self.estop = 0;
                self.robotRunning = 0;

                % input intermediary and final poses for book
                bookChemPoseIntermediary{1} = bookChemPose{1} * transl(0,-0.495,0);
                bookChemPoseFinal{1} = transl(-1.1,1.75,1.13) *troty(pi/2);

                penPoseIntermediary{1} = penPose{1} * transl(0,0,0.8);
                penPoseFinal{1} = transl(-0.1,1.75,0.945) * trotx(pi/2);

                pencilPoseIntermediary{1} = pencilPose{1} * transl(0,0,0.8);
                pencilPoseFinal{1} = transl(0,1.75,0.945) * trotx(pi/2);

                %% Adjust the position of robot and gripper
                r.model.animate([-0.5   -1.5708   -1.5708         pi/2         0         0         0])
                g1.model.base = g1.model.base.T * trotz(-pi/2);
                g2.model.base = g2.model.base.T * trotz(pi/2);

                %% Move the Engineering book back and forth using RMRC

                % Go from initial position to book pos
                startingQ = r.model.getpos();
                % InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
                InitialQ = [0.511   -1.5708   -1.0472    1.5708   150*pi/180         0         0];
                animateRMRC(self,r,g1,g2,0,r.model.fkine(startingQ).T,bookChemPose{1},InitialQ)

                % Close the gripper
                gripperAnimate(g1,g2,1);

                % Grab book from shelf
                InitialQ = [-0.5   -1.6982   -45*pi/180    60*pi/180         160*pi/180         0         0];
                animateRMRC(self,r,g1,g2,chemBookObj,bookChemPose{1},bookChemPoseIntermediary{1},InitialQ)

                InitialQ = r.model.getpos();
                % InitialQ = [0.1989   -2.0681   -0.4484    0.8277   -0.3699   -0.4955   -3.1411];
                animateRMRC(self,r,g1,g2,chemBookObj,bookChemPoseIntermediary{1},bookChemPoseFinal{1},InitialQ)

                % Open the gripper
                gripperAnimate(g1,g2,2);

                % Make Pulse75 go home
                InitialQ = [-0.5   -1.5708   -1.5708         pi/2         0         0         0];
                AnimateBricks(self,r,g1,g2,r.model.fkine(InitialQ).T,0,InitialQ);

                % Make Dobot pick up pen
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,penPose{1},0,InitialQ);

                InitialQ = d.model.getpos();
                AnimateBricks(self,d,0,0,penPoseIntermediary{1},penObj,InitialQ);

                InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
                AnimateBricks(self,d,0,0,penPoseFinal{1},penObj,InitialQ);

                % Make Dobot pick up pencil
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,pencilPose{1},0,InitialQ);

                InitialQ = d.model.getpos();
                AnimateBricks(self,d,0,0,pencilPoseIntermediary{1},pencilObj,InitialQ);

                InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
                AnimateBricks(self,d,0,0,pencilPoseFinal{1},pencilObj,InitialQ);

                % Make dobot go home
                InitialQ = [0 0 0 0 0];
                AnimateBricks(self,d,0,0,d.model.fkine(InitialQ).T,0,InitialQ);

            end

        end

        %% Maths mode
        function maths(self, robot)

            while self.startRobot == 0
                pause(1);
            end
            if self.startRobot == 1
                % load environment script, fills workspace with relevant variables
                EnvGen();
                r = robo75;
                d = roboDB;

                % properties from Animation class (used for only for testing purposes/debugging)
                self.estop = 0;
                self.robotRunning = 0;

                % input intermediary and final poses for book
                bookMathPoseIntermediary{1} = bookMathPose{1} * transl(0,-0.495,0);
                bookMathPoseFinal{1} = transl(-1.1,1.75,1.13) *troty(pi/2);

                penPoseIntermediary{1} = penPose{1} * transl(0,0,0.8);
                penPoseFinal{1} = transl(-0.1,1.75,0.945) * trotx(pi/2);

                compassPoseIntermediary{1} = compassPose{1} * transl(0,0,0.8);
                compassPoseFinal{1} = transl(0,1.75,0.945) * trotx(pi/2);

                %% Adjust the position of robot and gripper
                r.model.animate([-0.5   -1.5708   -1.5708         pi/2         0         0         0])
                g1.model.base = g1.model.base.T * trotz(-pi/2);
                g2.model.base = g2.model.base.T * trotz(pi/2);
                %% Move the Engineering book back and forth using RMRC

                InitialQ = r.model.getpos();
                animateRMRC(self,r,g1,g2,0,r.model.fkine(InitialQ).T,bookMathPose{1},InitialQ)

                % Close the gripper
                gripperAnimate(g1,g2,1);

                % Move book from shelf to table
                InitialQ = [-0.5   -1.6982   -45*pi/180    60*pi/180         160*pi/180         0         0];
                animateRMRC(self,r,g1,g2,mathBookObj,bookMathPose{1},bookMathPoseIntermediary{1},InitialQ)

                InitialQ = r.model.getpos();
                animateRMRC(self,r,g1,g2,mathBookObj,bookMathPoseIntermediary{1},bookMathPoseFinal{1},InitialQ)

                % Open the gripper
                gripperAnimate(g1,g2,2);

                % Make Pulse75 go home
                InitialQ = [-0.5   -1.5708   -1.5708         pi/2         0         0         0];
                AnimateBricks(self,r,g1,g2,r.model.fkine(InitialQ).T,0,InitialQ);

                % Make Dobot pick up pen
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,penPose{1},0,InitialQ);

                InitialQ = d.model.getpos();
                AnimateBricks(self,d,0,0,penPoseIntermediary{1},penObj,InitialQ);

                InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
                AnimateBricks(self,d,0,0,penPoseFinal{1},penObj,InitialQ);

                % Make Dobot pick up compass
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,compassPose{1},0,InitialQ);

                InitialQ = d.model.getpos();
                AnimateBricks(self,d,0,0,compassPoseIntermediary{1},compassObj,InitialQ);

                InitialQ = [-1.5708    1.0472    1.0472    1.0472         0];
                AnimateBricks(self,d,0,0,compassPoseFinal{1},compassObj,InitialQ);

                % Make dobot go home
                InitialQ = [0 0 0 0 0];
                AnimateBricks(self,d,0,0,d.model.fkine(InitialQ).T,0,InitialQ);


            end

        end


    end
end
