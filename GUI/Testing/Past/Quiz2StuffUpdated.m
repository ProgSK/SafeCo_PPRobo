classdef Quiz2StuffUpdated < handle
    %#ok<*NASGU>
    %#ok<*NOPRT>
    %#ok<*TRYNC>

    methods
        function self = Quiz2StuffUpdated()
            clf
            clc
            % self.Question1();
            % self.Question2();
            % self.Question3();
            % self.Question4();
            % self.Question5();
            % self.Question6();
        end
    end

    methods(Static)
        %% Define the D parameters for the 4-link manipulator
        function Question1()
            L1 = Link('d',0.5,'a',0.3,'alpha',pi/2,'qlim',[-pi pi]);
            L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
            L3 = Link('d',0,'a',0.2,'alpha', -pi/2,'qlim',[-pi pi]);
            L4 = Link('d',0.7,'a',0,'alpha',pi/2,'qlim',[-pi pi]);

            robot = SerialLink([L1 L2 L3 L4], 'name', 'my4LinkRobot');
            q = zeros(1,robot.n);
            q = [2.6971, -0.9426, -1.9063, -1.5640]

            pose = robot.fkine(q)
        end
        %% UR3 Question
        function Question2()
            baseTr = transl([0 0 0.8]); % change numbers depending on question
            r = UR3(baseTr);
            q = [0,pi/10,0,0,0,0];
            endEffector = r.model.fkine(q)
            endEffectorT = r.model.fkine(q).T
            % endEffectorG = eye(4);
            % endEffectorG(1:3,1) = endEffector.n;
            % endEffectorG(1:3,2) = endEffector.o;
            % endEffectorG(1:3,3) = endEffector.a;
            % endEffectorG(1:3,4) = endEffector.t;
            % endEffectorG
            % endEffectorCalc = endEffector * eye(4)
            % endEffectorTranspose = (endEffector)'
        end
        %% Baxter Distance between left/right
        function Question3()
            mdl_baxter
            qLeft = [0,pi/10,0,0,0,-pi/10,0];
            qRight = [0,-pi/10,0,0,0,pi/10,0];

            leftmatrix = left.fkine(qLeft)
            rightmatrix = right.fkine(qRight)

            leftpoint = transl(leftmatrix)
            rightpoint = transl(rightmatrix)

            distance = norm(rightpoint - leftpoint)
            fprintf('The distance %.4f mm\n', (distance * 1000));
        end
        %% DensoVM6083 - THE LOGIC IS WRONG SOMEWHERE
        function Question4()
            L1=Link('alpha',-pi/2,'a',0.180, 'd',0.475, 'offset',0, 'qlim',[deg2rad(-170), deg2rad(170)]);
            L2=Link('alpha',0,'a',0.385, 'd',0, 'offset',-pi/2, 'qlim',[deg2rad(-90), deg2rad(135)]);
            L3=Link('alpha',pi/2,'a',-0.100, 'd',0, 'offset',pi/2, 'qlim',[deg2rad(-80), deg2rad(165)]);
            L4=Link('alpha',-pi/2,'a',0, 'd',0.329+0.116, 'offset',0, 'qlim',[deg2rad(-185), deg2rad(185)]);
            L5=Link('alpha',pi/2,'a',0, 'd',0, 'offset',0, 'qlim',[deg2rad(-120), deg2rad(120)]);
            L6=Link('alpha',0,'a',0, 'd',0.09, 'offset',0, 'qlim',[deg2rad(-360), deg2rad(360)]);

            densoRobot = SerialLink([L1 L2 L3 L4 L5 L6],'name','Denso VM6083G');
            densoRobot.name = 'Denso VM6083G';

            % Change q variables
            q1 = [0, -0.6981, 0, 0.3491, 0.5236, 0.1745];
            q2 = [-0.1745, -0.6981, 2.094, 0.3491, 0.5236, 0.1745];

            steps = 50; % change Step Count here
            qmat = jtraj(q1, q2, steps)

            maxDist = 0;

            for i = 1:steps
                T = densoRobot.fkine(qmat(i, :));
                endEff = transl(T)
                laserNew = endEff + [0, 0, 6] % Add Laser Length
                distance = norm(laserNew)

                if distance > maxDist
                    maxDist = distance
                end
            end

            fprintf('The largest distance between the base and the laser is %.4f meters\n', maxDist);
        end
        %% Generate Quintic & Trapezoidal                                                        % 1 = Quintic Polynomial, 2 = Trapezoidal Velocity
        function Question5()
            steps = 100; % Change Steps here                                                                 % Specify no. of steps

            mdl_puma560
            qlim = p560.qlim;

            % Chage q variables
            q1 =  [-0.4382, 0.2842, -0.7392, -3.142, -0.455, -2.703];
            q2 =  [0.9113, -0.5942, -0.01781, 0, 0.612, -0.9113];

            qMatrix = jtraj(q1,q2,steps);

            s = lspb(0,1,steps);                                             	% First, create the scalar function
            qMatrixTrap = nan(steps,6);                                             % Create memory allocation for variables
            for i = 1:steps
                qMatrixTrap(i,:) = (1-s(i))*q1 + s(i)*q2;                   	% Generate interpolated joint angles
            end


            velocity = zeros(steps,6);
            velocityTrap = zeros(steps,6);
            % acceleration  = zeros(steps,6);
            % accelerationTrap  = zeros(steps,6);
            % maxDiff = 0;
            for i = 2:steps
                velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:)                          % Evaluate relative joint velocity
                velocityTrap(i,:) = qMatrixTrap(i,:) - qMatrixTrap(i-1,:)
                % acceleration(i,:) = velocity(i,:) - velocity(i-1,:);                    % Evaluate relative acceleration
                % accelerationTrap(i,:) = velocityTrap(i,:) - velocityTrap(i-1,:);
            end
            %
            diff_velocity = abs(velocity(:, 1) - velocityTrap(:, 1)) % 1 is joint position
            largest_diff = max(diff_velocity)

            fprintf('The largest difference between the two profiles for joint 1 is %.4f\n', largest_diff);

        end
        function Question6()
            mdl_puma560
            q = zeros(1,6);                                                    % Create a vector of initial joint angles
            scale = 0.5;
            workspace = [-2 2 -2 2 -0.05 2];                                   % Set the size of the workspace when drawing the robot
            p560.plot(q,'workspace',workspace,'scale',scale);                  % Plot the robot

            initialQ =  []; % insert given initial joint values
            goalTR = []; % insert given Pose
            qMatrix = p560.ikine(goalTR,'q0',initialQ)
        end
    end
end
