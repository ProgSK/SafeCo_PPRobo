classdef Quiz4Stuff < handle
    %#ok<*NASGU>
    %#ok<*NOPRT>
    %#ok<*TRYNC>

    methods
        function self = Quiz4Stuff()
            clc
            clf
            % self.Question1(); % Given a x DOF robot and y triangles
            % self.Question2(); % Load puma560, which pose is closest to singularity?
            self.Question3(); % Given collision detection ellipsoid, radii and surface
            % self.Question4(); % Create a UR5 robot. At q, what is the manipulability measure
            % self.Question5(); % mdl_planar, using RMRC find the MoM
            % self.Question6(); % Lab 5 Exercises Question 2, where you need to try to find where an intersection happens
        end
    end

    methods(Static)
        %% Given a x DOF robot and y triangles
        function Question1()
            dof = 5;
            triangles = 200;
            collisionChecks = dof*triangles;
            collisionChecks
        end
        %% Load puma560, which pose is closest to singularity?
        function Question2()
            mdl_puma560;
            qOption = cell(1,4);
            measureOfManip = zeros(4,1);
            threshold = 1;

            % Change these values
            qOption{1} = [0 2.2677 -2.7332 0 -0.9425 0];
            qOption{2} = [0 0.7954 3.1416 0 0.7854 0];
            qOption{3} = [0 -1.5708 -3.2416 0 0 0];
            qOption{4} = [0 0.7954 0 0 0.7 0];

            for i = 1:length(qOption)

                % Set the size of the workspace when drawing the robot
                workspace = [-1 1 -1 1 -1 1];
                scale = 0.5;

                % Plot the robot
                p560.plot(qOption{i},'workspace',workspace,'scale',scale);
                J = p560.jacob0(qOption{i});
                measureOfManip(i,1) = sqrt( det(J(1:3,:) * J(1:3,:)' ));
            end

            [M,I] = min(measureOfManip);
            disp(['Min MoM: ', num2str(M), ' at Index: ', num2str(I)])
        end
        %% Given collision detection ellipsoid, radii and surface
        function Question3()
            % Define the parameters of the ellipsoid
            center = [3, 2, -1];
            radii = [1, 2, 3];

            % Create the meshgrid
            [X, Y] = meshgrid(-10:1:10, -10:1:10);
            Z = X;
            
            %%%%%%%%%%%%%%%%%%%%%%% DO NOT CHANGE %%%%%%%%%%%%%%%%%%%%%%%%%%
            % Reshape the meshgrid data into a column vector
            points = [X(:), Y(:), Z(:)];

            % Calculate the number of points inside the ellipsoid
            num_points_inside = sum(((points - center) ./ radii).^2, 2) <= 0.9999;

            % Count the number of points inside the ellipsoid
            count_inside = sum(num_points_inside);

            % Extract points that are inside the ellipsoid
            points_inside_ellipsoid = points(num_points_inside, :);

            % Plot the ellipsoid
            figure;
            scatter3(points_inside_ellipsoid(:,1), points_inside_ellipsoid(:,2), points_inside_ellipsoid(:,3), 'b.', 'filled');
            hold on;
            [x, y, z] = ellipsoid(center(1), center(2), center(3), radii(1), radii(2), radii(3), 50);
            surf(x, y, z, 'FaceAlpha', 0.5);
            surf(X, Y, Z, 'FaceAlpha', 0.5);
            axis equal;
            title(['Number of Points Inside Ellipsoid: ', num2str(count_inside)]);
            xlabel('X');
            ylabel('Y');
            zlabel('Z');
            grid on;
        end
        %% Create a UR5 robot. At q, what is the manipulability measure
        function Question4()
            r = UR5;
            q = deg2rad([0,50,-85,-45,90,0]);

            J = r.model.jacob0(q);
            % jTranslation = J(1:3,:);

            % measureOfManip = sqrt(det(jTranslation*jTranslation'))
            measureOfManip = sqrt( det(J(1:3,:) * J(1:3,:)' ))
        end
        %% mdl_planar, using RMRC find the MoM
        function Question5()
            steps = 200;
            mdl_planar2;                                  % Load 2-Link Planar Robot

            T1 = eye(4)*transl(1.5,-0.35,0);       % First pose
            T2 = eye(4)*transl(1.53,-0.85,0);      % Second pose

            M = [1 1 zeros(1,4)];                         % Masking Matrix

            q1 = p2.ikine(T1,'q0', [0.2 0], 'mask', M);                    % Solve for joint angles
            q2 = p2.ikine(T2, 'q0', [0.2 0], 'mask', M);                    % Solve for joint angles
            p2.plot(q1,'trail','r-');
            pause(3)

            qMatrix = jtraj(q1,q2,steps);
            p2.plot(qMatrix,'trail','r-');

            % Get the jacobian for all q values and calculate the MoM
            trajJacob = zeros(length(qMatrix),6);
            measureOfManip = zeros(length(qMatrix),1);
            for i = 1:length(qMatrix)
                trajJacob = p2.jacob0(qMatrix(i,:));
                measureOfManip(i,:) = sqrt( det(trajJacob(1:2,:) * trajJacob(1:2,:)' ));
            end

            % Display the highest MoM and its index
            [maxM,maxI] = max(measureOfManip);
            disp(['Max MoM for jtraj: ', num2str(maxM), ' at Index: ', num2str(maxI)])

            % Resolved Motion Rate Control
            steps = 200;

            x1 = [1.5 -0.35]';
            x2 = [1.53 -0.85]';
            deltaT = 0.05;                                        % Discrete time step

            x = zeros(2,steps);
            s = lspb(0,1,steps);                                 % Create interpolation scalar
            for i = 1:steps
                x(:,i) = x1*(1-s(i)) + s(i)*x2;                  % Create trajectory in x-y plane
            end

            qMatrix = nan(steps,2);

            qMatrix(1,:) = p2.ikine(T1, 'q0', [0.2 0], 'mask', M);                 % Solve for joint angles

            for i = 1:steps-1
                xdot = (x(:,i+1) - x(:,i))/deltaT;                             % Calculate velocity at discrete time step
                J = p2.jacob0(qMatrix(i,:));            % Get the Jacobian at the current state
                J = J(1:2,:);                           % Take only first 2 rows
                qdot = inv(J)*xdot;                             % Solve velocitities via RMRC
                qMatrix(i+1,:) =  qMatrix(i,:) + deltaT*qdot';                   % Update next joint state
            end

            p2.plot(qMatrix,'trail','r-');

            % Get the jacobian for all q values and calculate the MoM
            rmrcJacob = zeros(length(qMatrix),6);
            measureOfManipRMRC = zeros(length(qMatrix),1);
            for i = 1:length(qMatrix)
                rmrcJacob = p2.jacob0(qMatrix(i,:));
                measureOfManipRMRC(i,:) = sqrt( det(rmrcJacob(1:2,:) * rmrcJacob(1:2,:)' ));
            end

            % Display the highest MoM and its index
            [maxM,maxI] = max(measureOfManipRMRC);
            disp(['Max MoM for jtraj: ', num2str(maxM), ' at Index: ', num2str(maxI)])
        end
        %% Lab 5 Exercises Question 2, where you need to try to find where an intersection happens
        function Question6()
            % Previous quiz 3 code
            mdl_3link3d
            robot = R3;
            q = [-pi/8,0,0];

            tr = zeros(4,4,robot.n+1);
            tr(:,:,1) = robot.base;
            L = robot.links;
            for i = 1 : robot.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end

            % % A plane can be defined with the following point and normal vector
            planeNormal = [-1,0,0];
            planePoint = [6,0,0];

            % Then if we have a line (perhaps a robot's link) represented by two points:
            lineStartPoint = [tr(1,4,2),tr(2,4,2),tr(3,4,2)];
            lineEndPoint = [tr(1,4,end),tr(2,4,end),tr(3,4,end)];

            [intersectionPoints,check] = LinePlaneIntersection(planeNormal,planePoint,lineStartPoint,lineEndPoint);
            intersectionPoints
        end
        
        function Question7()

        end

    end
end
