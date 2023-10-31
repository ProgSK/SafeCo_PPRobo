clear;
clf;
clc;
workspace = [-0.2, 0.2, -0.2, 0.2, -0.2, 0.2];
scale = 0.5;
q = [0 0 0];
%%
% link(1) = Link('d',0.04,'a',0.04,'alpha',pi/2,'qlim',[deg2rad(-20) deg2rad(20)]);
% link(2) = Link('d',0,'a',0.04,'alpha',0,'qlim',[deg2rad(-20) deg2rad(20)],'offset',pi/2);
% link(3) = Link('d',0,'a',0.06,'alpha',0,'qlim',[deg2rad(-20) deg2rad(20)]);
% 
% 
% model = SerialLink(link,'name','gripper');
% model.plot(q,'workspace',workspace,'scale',scale);

%%

g1TR = eye(4);
g1 = Gripper2F85(g1TR);
g2TR = trotz(pi);
g2 = Gripper2F85(g2TR);





% model.teach();

% g1.model.plot(q,'workspace',workspace,'scale',scale);
g1.model.teach();




