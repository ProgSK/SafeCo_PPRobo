%% Animate ikine
function AnimateBricks(self,robot,finger1,finger2,finalPose,obj,initialQ)
steps = 50;

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

    while self.estop == 1
        % this pauses the code while the estop is pressed
        pause(1);
        while self.robotRunning == 0
            pause(1); %continue button while loop
        end
    end
    
    if finger1 == 0 && finger2 == 0
        % Do nothing
    else
        finger1.model.base = endEffectorPose.T * trotx(-pi/2);
        finger1.model.animate(finger1.model.getpos());
        finger2.model.base = endEffectorPose.T * trotx(-pi/2) * troty(pi);
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
