function cloudBot(robot)
%Sample the joint angles within the joint limits at 30 degree increments between each of the joint limits
% Use fkine to determine the point in space for each of these poses, so that you end up with a big list of points
stepRads = deg2rad(45);
qlim = robot.model.qlim;
%joint 6 dw
pointCloudeSize = prod(floor((qlim(1:7,2)-qlim(1:7,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            for q4 = qlim(4,1):stepRads:qlim(4,2)
                for q5 = qlim(5,1):stepRads:qlim(5,2)
                    for q6 = qlim(6,1):stepRads:qlim(6,2)
                        % joint 6 dw assume 0
                        q7 = 0;
                        %for q6 = qlim(6,1):stepRads:qlim(6,2)
                        q = [q1,q2,q3,q4,q5,q6,q7];
                        tr = robot.model.fkineUTS(q);
                        pointCloud(counter,:) = tr(1:3,4)';
                        counter = counter + 1;
                        if mod(counter/pointCloudeSize * 100,1) == 0
                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                        end
                        %                     end
                    end
                end
            end
        end
    end
end

%Create a 3D model showing where the end effector can be over all these samples.
plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
end