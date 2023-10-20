clear;
clf;
clc;

%% Create a 3-link 3D robot with mdl_3link3d - gonna commit a felony if it dont work in quiz >:(
% mdl_3link3d
% robot = R3;
% q = [pi/6,0,0];
% 
% tr = zeros(4,4,robot.n+1);
% tr(:,:,1) = robot.base;
% L = robot.links;
% for i = 1 : robot.n
%     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
% end
% 
% % % A plane can be defined with the following point and normal vector
% planeNormal = [-1,0,0];
% planePoint = [3.1,0,0];
% 
% % Then if we have a line (perhaps a robot's link) represented by two points:
% lineStartPoint = [tr(1,4,2),tr(2,4,2),tr(3,4,2)];
% lineEndPoint = [tr(1,4,end),tr(2,4,end),tr(3,4,end)];
% 
% [intersectionPoints,check] = LinePlaneIntersection(planeNormal,planePoint,lineStartPoint,lineEndPoint);
% intersectionPoints


%% Create a 5DOF planar manipulator where all DH values of a are 1m - peasy like pi lemon? <:)
% L1 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L2 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L3 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L4 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% L5 = Link('d',0,'a',1,'alpha',0,'qlim',[-pi pi]);
% 
% robot = SerialLink([L1 L2 L3 L4 L5], 'name', 'my5LinkRobot');
% 
% q = zeros(1,robot.n);
% q =  deg2rad([45,-30,55,-45,0]);
% endEffector = robot.fkine(q)


%% Given 2 joint states create 100 step trajectory, with both Quintic & Trapezoidal - Worked for practice question, so if it dont work for quiz, angiiee >:<
% steps = 100; % Change Steps here                                                                 % Specify no. of steps
% 
% mdl_puma560                                                                 
% qlim = p560.qlim;                                                           
% 
% % Change q variables
% q1 = [pi/10, pi/7, pi/5, pi/3, pi/4, pi/6];
% q2 = [-pi/10, -pi/7, -pi/5, -pi/3, -pi/4, -pi/6];
% 
% qMatrix = jtraj(q1, q2, steps);
% 
% s = lspb(0, 1, steps); % First, create the scalar function
% qMatrixTrap = nan(steps, 6); % Create memory allocation for variables
% for i = 1:steps
%     qMatrixTrap(i,:) = (1-s(i))*q1 + s(i)*q2; % Generate interpolated joint angles
% end
% 
% velocity = zeros(steps, 6);
% velocityTrap = zeros(steps, 6);
% 
% for i = 2:steps
%     velocity(i,:) = qMatrix(i,:) - qMatrix(i-1,:); % Evaluate relative joint velocity
%     velocityTrap(i,:) = qMatrixTrap(i,:) - qMatrixTrap(i-1,:);
% end
% 
% % Calculate the maximum absolute velocity after the loop
% maxVelocity = max(abs(velocity(:)));
% maxVelocityTrap = max(abs(velocityTrap(:)));
% 
% fprintf('Maximum Absolute Velocity (QuinticPolynomial): %f\n', maxVelocity);
% fprintf('Maximum Absolute Velocity (Trapezoidal): %f\n', maxVelocityTrap);

%% Create a pumpa 560 with mdl_puma560. Find location of depth sensor. Read this question carefully :')
% mdl_puma560
% robot = p560;
% location = [1 1 1];
% q = deg2rad([0,45,-80,0,45,0]);
% distance = 1.8;
% endEffectorTR = robot.fkine(q).T;
% endEffectorTR = (endEffectorTR(1:3,4))';
% 
% state = 2; %State 1 = x, State 2 = y, State 3 = z
% 
% switch state
%     case 1
%         %% solve for x_2
%         negativeLocationX = endEffectorTR(1)-sqrt((distance.^2)-((location(2)-endEffectorTR(2)).^2)-((location(3)-endEffectorTR(3)).^2))
%         positiveLocationX = endEffectorTR(1)+sqrt((distance.^2)-((location(2)-endEffectorTR(2)).^2)-((location(3)-endEffectorTR(3)).^2))
%     case 2
%         %% solve for y_2
%         negativeLocationY = endEffectorTR(2)-sqrt((distance.^2)-((location(1)-endEffectorTR(1)).^2)-((location(3)-endEffectorTR(3)).^2))
%         positiveLocationY = endEffectorTR(2)+sqrt((distance.^2)-((location(1)-endEffectorTR(1)).^2)-((location(3)-endEffectorTR(3)).^2))
%     case 3
%         %% solve for z_2
%         negativeLocationZ = endEffectorTR(3)-sqrt((distance.^2)-((location(1)-endEffectorTR(1)).^2)-((location(2)-endEffectorTR(2)).^2))
%         positiveLocationZ = endEffectorTR(3)+sqrt((distance.^2)-((location(1)-endEffectorTR(1)).^2)-((location(2)-endEffectorTR(2)).^2))
% end

%% Create puma determine where a ray cast from the Z axis (the approach vector) of the end effector intersects with a planar wall 
% mdl_puma560
% robot = p560;
% q = [pi/20,0,-pi/2,0,0,0];
% endEffectorTR = robot.fkine(q).T;
% 
% % A plane can be defined with the following point and normal vector
% planeNormal = [-1,0,0];
% planePoint = [1.2,0,0];
% 
% endEffectorPosition = endEffectorTR(1:3, 4)'; % Extract translation part
% 
% % Define the ray (line) starting from the end effector's position along the Z-axis
% lineStartPoint = endEffectorPosition;
% lineEndPoint = endEffectorPosition + endEffectorTR(1:3, 3)'; % endEffectorTR(1:3, 3) = Z-axis direction, endEffectorTR(1:3, 2) = Y-axis direction, endEffectorTR(1:3, 1) = X-axis direction 
% 
% % Then we can use the function to calculate the point of
% % intersection between the line (line) and plane (obstacle)
% [intersectionPoints,check] = LinePlaneIntersection(planeNormal,planePoint,lineStartPoint,lineEndPoint);
% 
% % The returned values and their means are as follows:
% % (1) intersectionPoints, which shows the xyz point where the line
% intersectionPoints

