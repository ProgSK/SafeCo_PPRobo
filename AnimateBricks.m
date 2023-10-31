%% Animate ikine
function AnimateBricks(robot,finger1,finger2,initialPose,finalPose,obj,initialState)
steps = 50;

% perform ikine
q1 = robot.model.ikcon(initialPose);
q2 = robot.model.ikcon(finalPose);

%% Animation for loop for moving each initial and final poses for each brick

if initialState == 1
    % Animate the robot going to the initial to obj position
    q0 = robot.model.getpos();
    q1 = jtraj(q0,q1,steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal

    for j = 1:length(q1)
        robot.model.animate(q1(j,:));

        endEffectorPose = robot.model.fkine(robot.model.getpos());
        finger1.model.base = endEffectorPose.T;
        finger2.model.base = endEffectorPose.T;
        drawnow();
    end
end

q0 = robot.model.getpos();
pause(0.25);
% Animate the robot going to the final obj position
q2 = jtraj(q0,q2,steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal

for k = 1:length(q2)
    robot.model.animate(q2(k,:));
    endEffectorPose = robot.model.fkine(robot.model.getpos());
    
    finger1.model.base = endEffectorPose.T;
    finger1.model.animate(finger1.model.getpos());
    finger2.model.base = endEffectorPose.T;
    finger2.model.animate(finger2.model.getpos());

    % While the robot is moving, animate the brick movement
    obj.robotModel{1}.base = endEffectorPose.T; %* transl(0,0,0.10);
    obj.robotModel{1}.animate(0);
    drawnow();
end
end
