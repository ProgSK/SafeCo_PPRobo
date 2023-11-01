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
                        OrderNo = 1;
                        disp("Order 1 button pressed!");
                        %animate the movement for each robot
                        % for k = 1:4
                        %     self.animateRobot(self.robot.model, self.qMatrix{k}, cup{k}, cupMoving{k},person{k}, personMoving{k},OrderNo);
                        % end
                        %% call in new physics function

                        self.physics(self.robot);
                        self.orderReady = 0;
                        self.startRobot = 0;
                        self.robotRunning = 0;
                        %physics(self.robot.model)
                        %self.physics(self.robot.model);

                        disp("Order Number 1 delivered!");


                        %                         % input intermediary and final poses for book
                        % bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
                        % bookEngPoseFinal{1} = transl(-1.1,1.75,1.1) *troty(pi/2);
                        %
                        % % engBookInitialQ = zeros(1,r.model.n)
                        %
                        % %% Adjust the position of robot and gripper
                        % r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
                        % g1.model.base = g1.model.base.T * trotz(-pi/2);
                        % g2.model.base = g2.model.base.T * trotz(pi/2);
                        % %% Move the Engineering book back and forth using RMRC
                        %
                        % InitialQ = r.model.getpos();
                        % animateRMRC(r,g1,g2,0,r.model.fkine(InitialQ).T,bookEngPose{1},InitialQ)
                        % % Close the gripper
                        % gripperAnimate(g1,g2,1);
                        %
                        % InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
                        % animateRMRC(r,g1,g2,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)
                        %
                        % InitialQ = r.model.getpos();
                        % animateRMRC(r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)
                        % % AnimateBricks(robo75,g1,g2,bookEngPoseIntermediary{1},bookEngPoseFinal{1},engBookObj,0);
                        %
                        % InitialQ = r.model.getpos();
                        % animateRMRC(r,g1,g2,engBookObj,bookEngPoseFinal{1},bookEngPoseIntermediary{1},InitialQ)
                        %
                        % InitialQ = r.model.getpos();
                        % animateRMRC(r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPose{1},InitialQ)
                        %
                        % % Open the gripper
                        % gripperAnimate(g1,g2,2);
                        %
                        %                         disp("Order Number 1 delivered!");


                        % case 2
                        %     %animate the movement for each robot
                        %     OrderNo = 2;
                        %     for l = 5:8
                        %         self.animateRobot(self.robot.model, self.qMatrix{l}, cup{l}, cupMoving{l},person{l}, personMoving{l},OrderNo);
                        %     end
                        %     self.orderReady = 0;
                        %     self.startRobot = 0;
                        %     self.robotRunning = 0;
                        %     disp("Order Number 2 delivered!");
                        % case 3
                        %     OrderNo = 3;
                        %     %animate the movement for each robot
                        %     for h = 9:12
                        %         self.animateRobot(self.robot.model, self.qMatrix{h}, cup{h}, cupMoving{h},person{h}, personMoving{h},OrderNo);
                        %         %     pause
                        %     end
                        %     self.orderReady = 0;
                        %     self.startRobot = 0;
                        %     self.robotRunning = 0;
                        %     disp("Order Number 3 delivered!");
                end
            end
            %------------------------------------------------

        end


        % function qMatrix = transformMoves(~,robot, cupTR)
        %     %TRANSFORMMOVES This function calculates the qMatrix
        %     %   This function takes the robot in use and calculates the
        %     %   Trapezoidal Velocity Profile qMatrix
        %
        %     % We ran out of time to replace this with RMRC or quintic
        %     % polynomial
        %     % Trap completes the movement quickly however it has jerk and
        %     % so is not appropriate for the use case. An alternative would
        %     % be quintic or s-curve trajectory planning
        %
        %     steps = 50; %%more steps ->slower code and movement
        %     joints=7;
        %     qCurrent = zeros(1,joints);
        %     iterations = 12; %%number of moves. change for number of moves required
        %     qMatrix = cell(iterations, joints);
        %     for i = 1:iterations
        %         if cupTR{i} == 0
        %             qMatrix{i} = 0;
        %         else
        %             qGoal = robot.model.ikcon(cupTR{i}, qCurrent); % include joint limits with ikone
        %             s = lspb(0, 1, steps); %polynomial distance from 0 to 1
        %             qMatrix{i} = zeros(steps, joints);
        %
        %             for j = 1:steps
        %                 qMatrix{i}(j, :) = (1-s(j))*qCurrent + s(j)*qGoal;
        %             end
        %             qCurrent = qGoal;
        %         end
        %     end
        % end


        % %         function animateRobot()
        % %             %ANIMATEROBOTS This function makes the robot move
        % %             %   This function takes the robot in use, Trapezoidal Velocity Profile,
        % %             %   cup in use and whether the cup is to also be moved. It then uses
        % %             %   animate to move the robot.
        % %             %   when an order is ready it also moves the correct person
        % %             %   forward to collect the cup
        % %
        % %             % hard coded in get moves
        % %
        % %             while self.startRobot == 0
        % %                 pause(1);
        % %             end
        % %             if self.startRobot == 1
        % %                 % steps = height(qMatrix);
        % %                 % for i = 1:steps
        % %                 %
        % %                 %     %animate robot motion
        % %                 %     if size(qMatrix) > 1
        % %                 %         animate(robot, qMatrix(i, :));
        % %                 %     end
        % %                 %
        % %                 %     %animate cup motion
        % %                 %     if cupMoving == true
        % %                 %         newPos1 = robot.fkine(qMatrix(i, :)).T; %find the robot ee position
        % %                 %         newPos1 = newPos1*transl(0,0,-0.1); %drop the cup location slightly just cause it didnt work on my computer?
        % %                 %         ee = newPos1(1:3,4); % THIS IS WHERE WE MASK THE CUP YAW SO IT IS ALWAYS UPRIGHT
        % %                 %         cup.updatePosition(transl(ee));
        % %                 %     end
        % %                 %
        % %                 %     %% animate person motion
        % %                 %     % if personMoving == true
        % %                 %     %     %self.personEndLocation{i}
        % %                 %     %     if i == 1
        % %                 %     %         currentPos = self.personStartLocation{OrderNo};
        % %                 %     %         currentPos = currentPos(1:3,4);
        % %                 %     %         cupPos = transl( 0.5 ,0, 1.4); %%hard coded hand off position
        % %                 %     %         cupPos = cupPos(1:3,4);
        % %                 %     %
        % %                 %     %     end
        % %                 %     %
        % %                 %     %     %For order 1
        % %                 %     %     if OrderNo == 1
        % %                 %     %         % if i <= 25 % person moving towards the cup
        % %                 %     %         %     matrix = [0.9/25;0;0]; %step the person is moving
        % %                 %     %         % end
        % %                 %     %         %
        % %                 %     %         % if i > 25 %person moving back with the cup
        % %                 %     %         %     matrix = [-0.9/25;0;0]; %step the person is moving
        % %                 %     %         %     cupPos = cupPos+matrix;
        % %                 %     %         %     tester = transl(cupPos);%*transl(0,0,-0.25); %added a transl down so the cup is in hand
        % %                 %     %         %     cup.updatePosition(tester); %moved the cup position by the same step as the person
        % %                 %     %         % end
        % %                 %     %         % currentPos = currentPos+matrix;
        % %                 %     %         % personPos = transl(currentPos);
        % %                 %     %         % person.updatePosition(personPos);
        % %                 %     %         disp("Order 1 button pressed!");
        % %                 %     %
        % %                 %     %     end
        % %                 %     %
        % %                 %     % end
        % %
        % %                 %% Move the Engineering book back and forth using RMRC
        % % % input intermediary and final poses for book
        % %             bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
        % %             bookEngPoseFinal{1} = transl(-1.1,1.75,1.1) *troty(pi/2)
        % %
        % %             %% Adjust the position of robot and gripper
        % %             r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
        % %             g1.model.base = g1.model.base.T * trotz(-pi/2);
        % %             g2.model.base = g2.model.base.T * trotz(pi/2);
        % %
        % % InitialQ = r.model.getpos();
        % % animateRMRC(r,g1,g2,0,r.model.fkine(InitialQ).T,bookEngPose{1},InitialQ)
        % % % Close the gripper
        % % gripperAnimate(g1,g2,1);
        % %
        % % InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
        % % animateRMRC(r,g1,g2,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)
        % %
        % % InitialQ = r.model.getpos();
        % % animateRMRC(r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)
        % % % AnimateBricks(robo75,g1,g2,bookEngPoseIntermediary{1},bookEngPoseFinal{1},engBookObj,0);
        % %
        % % InitialQ = r.model.getpos();
        % % animateRMRC(r,g1,g2,engBookObj,bookEngPoseFinal{1},bookEngPoseIntermediary{1},InitialQ)
        % %
        % % InitialQ = r.model.getpos();
        % % animateRMRC(r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPose{1},InitialQ)
        % %
        % % % Open the gripper
        % % gripperAnimate(g1,g2,2);
        % %
        % %                     while self.estop == 1
        % %                         % this pauses the code while the estop is pressed
        % %                         pause(1);
        % %                         while self.robotRunning == 0
        % %                             pause(1); %continue button while loop
        % %                         end
        % %                     end
        % %                 %
        % %                 %     drawnow()
        % %                 % end
        % %
        % %             end
        % %         end


        function physics(self, robot)

            while self.startRobot == 0
                pause(1);
            end
            if self.startRobot == 1
                % load environment script, fills workspace with relevant variables
                EnvGen();
                r = robot;
                r = robo75;
                d = roboDB;

                % input intermediary and final poses for book
                bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
                bookEngPoseFinal{1} = transl(-1.1,1.75,1.1) * troty(pi/2);

                penPoseIntermediary{1} = penPose{1} * transl(0,0,0.8);
                penPoseFinal{1} = transl(-0.1,1.75,0.945) * trotx(pi/2);

                rulerPoseIntermediary{1} = rulerPose{1} * transl(0,0,0.8);
                rulerPoseFinal{1} = transl(0,1.75,0.945) * trotx(pi/2);

                %% Adjust the position of robot and gripper
                r.model.animate([0   -1.5708   -1.5708         0         0         0         0])
                g1.model.base = g1.model.base.T * trotz(-pi/2);
                g2.model.base = g2.model.base.T * trotz(pi/2);
                %% Move the Engineering book back and forth using RMRC
                

                % GO from initial position to book pos
                InitialQ = r.model.getpos();
                animateRMRC(self,r,g1,g2,0,r.model.fkine(InitialQ).T,bookEngPose{1},InitialQ)

                

                % Close the gripper
                gripperAnimate(g1,g2,1);

                

                % Grab book from shelf
                InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
                animateRMRC(self,r,g1,g2,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)

               

                InitialQ = r.model.getpos();
                animateRMRC(self,r,g1,g2,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)
                
              

                % Open the gripper
                gripperAnimate(g1,g2,2);

               

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
                InitialQ = [0    0.7854    0.7854    1.5708         0];
                AnimateBricks(self,d,0,0,rulerPose{1},0,InitialQ);

                
                drawnow()
            end

        end
    end
end
