%% Inverse kinematics function
function qMatrix = RobotIK(robot, Pose)
objNo = numel(Pose);
qMatrix = cell(1,objNo);
% tr = zeros(4,4,robot.model.n+1);
for i = 1:objNo
    endEffectorPose = Pose{i}; %* transl(0, 0, -0.10);
    qMatrix{i} = robot.model.ikcon(endEffectorPose);

    % % 2.4: Get the transform of every joint (i.e. start and end of every link)
    % tr(:,:,1) = robot.model.base;
    % L = robot.model.links;
    % for j = 1 : robot.model.n
    %     tr(:,:,j+1) = tr(:,:,j) * trotz(qMatrix{i}(1,j)+L(j).offset) * transl(0,0,L(j).d) * transl(L(j).a,0,0) * trotx(L(j).alpha);
    % end
    % if tr(3,4,2) < 0.3
    %     qMatrix{i}(1,2) = -qMatrix{i}(1,2);
    % end
    
end
end