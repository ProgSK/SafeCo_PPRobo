clear;
clf;
clc;

EnvGen();
r = robo75;
%% Animate using RMRC

% pencilInitialQ = [0    0.8836   -0.8378    1.7453   -2.4544   -1.5217         0]; % From teach pendant manual movement (eyeballing)
% pencilInitialQ = zeros(1,r.model.n);
% animateRMRC(r,pencilObj,pencilPose{1},pencilFinalPose{1},pencilInitialQ)
bookEngPoseIntermediary{1} = bookEngPose{1} * transl(0,-0.495,0);
bookChemPoseIntermediary{1} = bookChemPose{1} * transl(0,-0.495,0);
bookMathPoseIntermediary{1} = bookMathPose{1} * transl(0,-0.495,0);

bookEngPoseFinal{1} = transl(-1,1.25,1.1) *troty(pi/2);

% engBookInitialQ = zeros(1,r.model.n)

% Move the Engineering book back and forth
InitialQ = [0   -1.6982   -2.2076    0.5535         0         0         0];
animateRMRC(r,engBookObj,bookEngPose{1},bookEngPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,engBookObj,bookEngPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,engBookObj,bookEngPoseFinal{1},bookEngPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,engBookObj,bookEngPoseIntermediary{1},bookEngPose{1},InitialQ)

% Move the Maths book back and forth
InitialQ = r.model.getpos();
animateRMRC(r,mathBookObj,bookMathPose{1},bookMathPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,mathBookObj,bookMathPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,mathBookObj,bookEngPoseFinal{1},bookMathPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,mathBookObj,bookMathPoseIntermediary{1},bookMathPose{1},InitialQ)

% Move the Chemistry book back and forth
InitialQ = r.model.getpos();
animateRMRC(r,chemBookObj,bookChemPose{1},bookChemPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,chemBookObj,bookChemPoseIntermediary{1},bookEngPoseFinal{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,chemBookObj,bookEngPoseFinal{1},bookChemPoseIntermediary{1},InitialQ)

InitialQ = r.model.getpos();
animateRMRC(r,chemBookObj,bookChemPoseIntermediary{1},bookChemPose{1},InitialQ)

%% Find q values of all object poses
% qMatrixPencilInitial = RobotIK(r,pencilPose);
% qMatrixPencilFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixPenInitial = RobotIK(r,penPose);
% qMatrixPenFinal = RobotIK(r,penFinalPose);

% qMatrixPencilInitial = RobotIK(r,pencilPose);
% qMatrixPencilFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixRulerInitial = RobotIK(r,pencilPose);
% qMatrixRulerFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixCalcInitial = RobotIK(r,pencilPose);
% qMatrixCalcFinal = RobotIK(r,pencilFinalPose);
% 
% qMatrixEraserInitial = RobotIK(r,pencilPose);
% qMatrixEraserFinal = RobotIK(r,pencilFinalPose);

%% Interpolate and animate the joint values
% AnimateBricks(r,qMatrixPencilInitial,qMatrixPencilFinal,pencilObj,steps);
% AnimateBricks(r,qMatrixPenInitial,qMatrixPenFinal,penObj,steps);


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

%% Animate ikine
function AnimateBricks(robot,qMatrixInitial,qMatrixFinal,obj,steps)

% Set up variables
noPose = numel(qMatrixInitial);

% Animation for loop for moving each initial and final poses for each brick
for i = 1:noPose

    % Animate the robot going to the initial brick (i) position
    q1 = robot.model.getpos();
    q1 = jtraj(q1,qMatrixInitial{i},steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal

    for j = 1:length(q1)
        robot.model.animate(q1(j,:));
        drawnow();
    end

    % Display current progress and difference between initial brick pose and initial calculated pose
    q1 = robot.model.getpos();
    pause(0.25);
    % Animate the robot going to the final brick (i) position
    q2 = jtraj(q1,qMatrixFinal{i},steps); % Minimum jerk trajectory, which takes the form of a quintic polynominal
    for k = 1:length(q2)
        robot.model.animate(q2(k,:));
        endEffectorPose = robot.model.fkine(robot.model.getpos());

        % While the robot is moving, animate the brick movement
        obj.robotModel{i}.base = endEffectorPose.T; %* transl(0,0,0.10);
        obj.robotModel{i}.animate(0);
        drawnow();
    end
end
end