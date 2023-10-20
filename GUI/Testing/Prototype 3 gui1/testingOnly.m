% mdl_puma560
% robot = p560;
% q = zeros(1,3);
% robot.teach

% mdl_puma560
% robot = p560;
% q = [0,0,0,0,0,0]
% robot.teach

mdl_puma560
q = zeros(1,6);                                                    % Create a vector of initial joint angles
scale = 0.5;
workspace = [-2 2 -2 2 -0.05 2];                                   % Set the size of the workspace when drawing the robot
p560.plot(q,'workspace',workspace,'scale',scale);
