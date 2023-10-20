% adding function MyAssessment1() causes the robot to dissapear
clear;
clc;
clf;

axis on;
%change values here for workspace dimensions & in SpwanBricksRobo line 27 
%axis ([-4 4 -4 5 0 4]);
axis auto
view(3);

%% Modular everything
% arrVal will be the x,y,z values for positioning EVERYTHING pog
arrVal = [0,0,0];

%% Kinda like frames of the animation
fpsIsh = 50;

%% Calling in the functions to create the environment
floor(arrVal)
envi(arrVal)

%% My robot's position
baseTR = transl([arrVal(1),arrVal(2),arrVal(3)+0.5]);
myUR3 = LinearUR3(baseTR);

%% Point cloud run when required, takes time patience 30sec ish :>
% cloudBot(myUR3)

%% The starting and ending positions of the bricks
% note: trotx(pi) was used to flip the brick from vertical to flat
% note: trotz(pi/2) was used to rotate the brick to line up the bricks from the longest lenght of one brick to another

% Starting postion of each brick is saved in a cell
brickStartpos = cell(1,9); % row 1 with 9 cells for 9 bricks
brickStartpos{1} = transl(-0.1+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{2} = transl(-0.2+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{3} = transl(-0.3+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{4} = transl(-0.4+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{5} = transl(-0.5+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{6} = transl(-0.6+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{7} = transl(-0.7+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{8} = transl(-0.8+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);
brickStartpos{9} = transl(-0.9+arrVal(1),+0.45+arrVal(2),0.54+arrVal(3))* trotx(pi);

% Ending postion of each brick is saved in a cell too
% Below code for a NO gap in the z direction to show that bricks are being placed on top PERFECTLY
brickEndpos = cell(1,9); % row 1 with 9 cells for 9 bricks
brickEndpos{1} = transl(-0.1  +arrVal(1),-0.45+arrVal(2),0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{2} = transl(-0.235+arrVal(1),-0.45+arrVal(2),0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{3} = transl(-0.37 +arrVal(1),-0.45+arrVal(2),0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{4} = transl(-0.1  +arrVal(1),-0.45+arrVal(2),0.034+0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{5} = transl(-0.235+arrVal(1),-0.45+arrVal(2),0.034+0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{6} = transl(-0.37 +arrVal(1),-0.45+arrVal(2),0.034+0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{7} = transl(-0.1  +arrVal(1),-0.45+arrVal(2),0.034+0.034+0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{8} = transl(-0.235+arrVal(1),-0.45+arrVal(2),0.034+0.034+0.54+arrVal(3))* trotx(pi)*trotz(pi/2);
brickEndpos{9} = transl(-0.37 +arrVal(1),-0.45+arrVal(2),0.034+0.034+0.54+arrVal(3))* trotx(pi)*trotz(pi/2);

% Below code for a little gap in the z direction to show that bricks are being placed on top and not fusing into eachoter 
%brickEndpos = cell(1,9); % row 1 with 9 cells for 9 bricks
% brickEndpos{1} = transl(-0.1+arrVal(1),-0.45+arrVal(2),0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{2} = transl(-0.235+arrVal(1),-0.45+arrVal(2),0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{3} = transl(-0.37+arrVal(1),-0.45+arrVal(2),0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{4} = transl(-0.1+arrVal(1),-0.45+arrVal(2),0.035+0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{5} = transl(-0.235+arrVal(1),-0.45+arrVal(2),0.035+0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{6} = transl(-0.37+arrVal(1),-0.45+arrVal(2),0.035+0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{7} = transl(-0.1+arrVal(1),-0.45+arrVal(2),0.035+0.035+0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{8} = transl(-0.235+arrVal(1),-0.45+arrVal(2),0.035+0.035+0.54+arrVal(3))* trotx(pi)*trotz(pi/2)
% brickEndpos{9} = transl(-0.37+arrVal(1),-0.45+arrVal(2),0.035+0.035+0.54+arrVal(3))* trotx(pi)*trotz(pi/2)



%% Spawning bricks into the workspace W :)
% refer to SpawnBricksRobo class 

spawnBricks = SpawnBricksRobo(9,brickStartpos);



%% Finding q values of the bricks and animating the joint values
% see InvKinBrick function
% performing inverse kinematics calcluations from brick start position to end positon
qBrickStarting = InvKBrick(myUR3,brickStartpos);
qBrickEnding = InvKBrick(myUR3,brickEndpos);

% COME ALIVEEEE!
motionBricks(myUR3,qBrickStarting,qBrickEnding,spawnBricks,fpsIsh);


%% Movement of the bricks
function qBrickMat = InvKBrick(robot,brickPos) %brickPos end for ending, start for starting
qBrickMat = cell(1,9); %1 to 9 because 9 places for 9 bricks

for i = 1:9
    moveEndEffector = brickPos{i}; %*transl(0, 0, 0.15)
    qBrickMat{i} = robot.model.ikcon(moveEndEffector);
end
%% GRIPPER WHY SO HARD :<
%*transl(0, 0, 0.15) USE this for GRIPPER when you make Link 7 ply file
end

%% Creating the floor for the environment
function floor(arrVal)
x=4;
surf([-x+arrVal(1),-x+arrVal(1);  x+arrVal(1), x+arrVal(1)] ...
    ,[-x+arrVal(2), x+arrVal(2); -x+arrVal(2), x+arrVal(2)] ...
    ,[arrVal(3) + 0.01, arrVal(3) + 0.01; arrVal(3) + 0.01, arrVal(3) + 0.01] ...
    ,'CData',imread('floor.jpg') ...
    ,'FaceColor','texturemap');
hold on;

%input('Environment Ready. Initalising Robot. Press Enter.');
end

%% Robot: Putting robot as a function causes it to dissapear, don't use :<
function robot()
LinearUR3
% % adding input line makes robot stay until enter is pressed
% % input('Robot Ready. Placing Bricks. Press Enter');
end

%% Initalising environment
function envi(arrVal)
% Calling in my fence
%PlaceObject('fenceSigns.ply',[arrVal(1),arrVal(2),arrVal(3)+0.41]);

% Calling in my signs on the fence
%PlaceObject('fourSigns.ply',[arrVal(1),arrVal(2),arrVal(3)+0.41]);

% Calling in the engineer person
%PlaceObject('me.ply',[arrVal(1)-2,arrVal(2)-3,arrVal(3)+0.58]);

% Calling in the desk setup with chair, table and PC 
%PlaceObject('deskSetup.ply',[arrVal(1)+2,arrVal(2)-3.5,arrVal(3)+0.335]);

% Calling in Emergency Equipment fire extingusiher
%PlaceObject('fireExtinguisherSmol.ply',[arrVal(1)-1,arrVal(2)-2.9,arrVal(3)+0.04]);

% Calling in Emergency stop button
%PlaceObject('emergencyStopWallMounted.ply',[arrVal(1)-1,arrVal(2)-2.9,arrVal(3)+0.6]);

% Calling in the table for robot and bricks
PlaceObject('tableBrown2.1x1.4x0.5m.ply',[arrVal(1),arrVal(2),arrVal(3)]);

end

%% Used for test purposes :)
% Calling in bricks manually

function getBricks(arrVal)
b_1 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.8,arrVal(2)+0.45,arrVal(3)+0.5]);
b_2 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.7,arrVal(2)+0.45,arrVal(3)+0.5]);
b_3 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.6,arrVal(2)+0.45,arrVal(3)+0.5]);
b_4 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.5,arrVal(2)+0.45,arrVal(3)+0.5]);
b_5 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.4,arrVal(2)+0.45,arrVal(3)+0.5]);
b_6 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.3,arrVal(2)+0.45,arrVal(3)+0.5]);
b_7 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.2,arrVal(2)+0.45,arrVal(3)+0.5]);
b_8 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)-0.1,arrVal(2)+0.45,arrVal(3)+0.5]);
b_9 = PlaceObject('HalfSizedRedGreenBrick.ply',[arrVal(1)+0,arrVal(2)+0.45,arrVal(3)+0.5]);
end

%% OMG I SWEAR TO GOD Animate my robot rn >:/
function motionBricks(robot, qBrickStarting, qBrickEnding, spawnBricks, fpsIsh)

for n = 1:9
    q1 = robot.model.getpos();
    q1 = jtraj(q1,qBrickStarting{n},fpsIsh);
    qBrickMat = q1;

    for m = 1:length(qBrickMat)
        robot.model.animate(qBrickMat(m,:))
        drawnow();
    end

    q1 = robot.model.getpos()
    position = robot.model.fkine(q1)

    %Brick positions
    ActualPos = spawnBricks.cowModel{n}.base.t
    pause(0.25)


    q2 = jtraj(q1,qBrickEnding{n},fpsIsh);

    for v = 1:length(q2)
        robot.model.animate(q2(v,:));
        drawnow();

        %this is gonna be the end of me, end effector animation
        endMeJK = robot.model.fkine(robot.model.getpos());

        spawnBricks.cowModel{n}.base = endMeJK.T;
        spawnBricks.cowModel{n}.animate(0);
        drawnow()

    end
    pause(0);

end
end

