%% Robotics
% Updated syntax and tested for V10 compatibility (Jan 2023)
% Further updated by Sully for 7 DOF

% Lab 9 - Question 1 - Resolved Motion Rate Control in 6DOF
function animateRMRC(self,robot,finger1,finger2,obj,initialPose,finalPose,q0)
% 1.1) Set parameters for the simulation
r = robot;        % Load robot model
model = r.model;
t = 10;             % Total time (s)
deltaT = 0.2;      % Control frequency
steps = t/deltaT;   % No. of steps for simulation = 50 (Allows leniency for q0 estimates)
delta = 2*pi/steps; % Small angle change
epsilon = 0.1;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

initialTR = initialPose(1:3,4)';
goalTR = finalPose(1:3,4)';
eulXYZ = rotm2eul(finalPose(1:3,1:3),'XYZ');

% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,7);       % Array for joint anglesR
qdot = zeros(steps,7);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % For plotting trajectory error
angleError = zeros(3,steps);    % For plotting trajectory error

% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);                % Trapezoidal trajectory scalar
for i=1:steps

    x(1,i) = (1-s(i))*initialTR(1) + s(i)*goalTR(1); % Points in x
    x(2,i) = (1-s(i))*initialTR(2) + s(i)*goalTR(2); % Points in y
    x(3,i) = (1-s(i))*initialTR(3) + s(i)*goalTR(3); % Points in z
    theta(1,i) = -pi/2;                 % Roll angle
    theta(2,i) = 0;                 % Pitch angle
    theta(3,i) = 0;                 % Yaw angle
end

T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation of first point and angle
qMatrix(1,:) = model.ikcon(T,q0);                                            % Solve joint angles to achieve first waypoint

% for i = 1:length(qMatrix)
%     if qMatrix(i,1) < 0
%         qMatrix(i,1) = 0.01;
%     end
%     if qMatrix > 5
%         qMatrix(i,1) = 5;
%     end
% end

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    % UPDATE: fkine function now returns an SE3 object. To obtain the
    % Transform Matrix, access the variable in the object 'T' with '.T'.
    T = model.fkine(qMatrix(i,:)).T;                                         % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = model.jacob0(qMatrix(i,:));                                          % Get Jacobian at current joint state
    m(i) = sqrt(det(J*J'));
    if m(i) < epsilon                                                       % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(7))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:6                                                             % Loop through joints 1 to 6
        if qMatrix(i,j) + deltaT*qdot(i,j) < model.qlim(j,1)                 % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > model.qlim(j,2)             % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

% 1.5) Plot the results
tic

% figure(1)
hold on
% plot3(x(1,:),x(2,:),x(3,:),'k.','LineWidth',1) % Plots straight line from initial to goal (remove in final release)

for k = 1:length(qMatrix)
    model.animate(qMatrix(k,:));
    endEffectorPose = robot.model.fkine(robot.model.getpos());

    while self.estop == 1
        % this pauses the code while the estop is pressed
        pause(1);
        while self.robotRunning == 0
            pause(1); %continue button while loop
        end
    end
    
    if obj == 0
        % Do nothing
    else
        % While the robot is moving, animate the obj movement
        % obj.robotModel{1}.base = endEffectorPose.T; % do not uncomment, causes issues with rotation 
        obj.robotModel{1}.base.t = endEffectorPose.t;
        obj.robotModel{1}.animate(0);
    end
    
    if finger1 == 0 && finger2 == 0
        % Do nothing
    else
        % Also update the base location of the gripper
        finger1.model.base.t = endEffectorPose.t;
        % finger1.model.base = endEffectorPose.T; % do not uncomment, causes issues with rotation 
        finger1.model.animate(finger1.model.getpos());

        finger2.model.base.t = endEffectorPose.t;
        % finger2.model.base = endEffectorPose.T; % do not uncomment, causes issues with rotation 
        finger2.model.animate(finger2.model.getpos());
    end

    drawnow();
    % pause(0.1);   % Slow-mo
end
disp(['Plot took ', num2str(toc), 'seconds'])

% figure(2)
% subplot(2,1,1)
% plot(positionError'*1000,'LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Position Error (mm)')
% legend('X-Axis','Y-Axis','Z-Axis')
% 
% subplot(2,1,2)
% plot(angleError','LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Angle Error (rad)')
% legend('Roll','Pitch','Yaw')
% figure(5)
% plot(m,'k','LineWidth',1)
% refline(0,epsilon)
% title('Manipulability')
% end

%% Plots for visualising manipulability
% figure(2)
% subplot(2,1,1)
% plot(positionError'*1000,'LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Position Error (mm)')
% legend('X-Axis','Y-Axis','Z-Axis')
%
% subplot(2,1,2)
% plot(angleError','LineWidth',1)
% refline(0,0)
% xlabel('Step')
% ylabel('Angle Error (rad)')
% legend('Roll','Pitch','Yaw')
% figure(5)
% plot(m,'k','LineWidth',1)
% refline(0,epsilon)
% title('Manipulability')