%% Animate ikine
function AnimateBricks(robot,finger1,finger2,finalPose,obj,initialQ)
steps = 50;

% perform ikine
% q1 = robot.model.ikcon(initialPose);
% q2 = robot.model.ikcon(finalPose);

if initialQ == 0
    q1 = robot.model.ikcon(finalPose);
else
    q1 = robot.model.ikunc(finalPose,initialQ);
end

%% Animation for loop 

q0 = robot.model.getpos();
pause(0.25);
% Animate the robot going to the final obj position
q1 = jtraj(q0,q1,steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal

for k = 1:length(q1)
    robot.model.animate(q1(k,:));
    endEffectorPose = robot.model.fkine(robot.model.getpos());
    
    if finger1 == 0 && finger2 == 0
        % Do nothing
    else
        finger1.model.base = endEffectorPose.T;
        finger1.model.animate(finger1.model.getpos());
        finger2.model.base = endEffectorPose.T;
        finger2.model.animate(finger2.model.getpos());
    end

    if obj == 0
        % Do nothing
    else
        % While the robot is moving, animate the brick movement
        obj.robotModel{1}.base = endEffectorPose.T; %* transl(0,0,0.10);
        obj.robotModel{1}.animate(0);
    end
    drawnow();
end
end
