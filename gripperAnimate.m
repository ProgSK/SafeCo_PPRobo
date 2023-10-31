function gripperAnimate(finger1,finger2,state)

closedState = deg2rad([-20 0 0]);
openState = [0 0 0];
steps = 10;

switch state
    case 1
        q1 = openState;
        q2 = closedState;
        qMatrix = jtraj(q1,q2,10);

        for i = 1:length(qMatrix)
            finger1.model.animate(qMatrix(i,:));
            finger2.model.animate(qMatrix(i,:));
            drawnow()
        end

    case 2
        q1 = closedState;
        q2 = openState;
        qMatrix = jtraj(q1,q2,10);

        for i = 1:length(qMatrix)
            finger1.model.animate(qMatrix(i,:));
            finger2.model.animate(qMatrix(i,:));
            drawnow()
        end
end