%SIMULATIONGUI Robot passing coffee GUI
%
% GUI::
%
% - The specified callback function is invoked every time the joint configuration changes.
%   the joint coordinate vector.
% - The Quit (red X) button destroys the teach window.

function getSimulationGUI(self)
    % f = findobj(gca, 'Tag', self.robot.model.name);
	% if isempty(f)
	% 	f = findobj(0, 'Tag', self.robot.model.name);
	% 	if isempty(f)
		% 	self.robot.model.plot(zeros(1, self.robot.model.n));
		% 	ax = gca;
	% 	else
		% 	ax = get(c(1), 'Parent');
	% 	end
	% else
	 ax = gca;
	% end
	
	bgcol = [1, 0.75, 0.50];
	handles.fig = get(ax, 'Parent');
%	set(ax, 'Outerposition', [0.3 0 0.8 1]);
%	handles.curax = ax;
	
	%GUI panel
	panel = uipanel(handles.fig, ...
		'Title', 'Focus Modes', ...
		'BackgroundColor', bgcol, ...
        'ForegroundColor', 'white', ...
		'Position', [0, 0.2, 0.25, 0.5]);
	set(panel, 'Units', 'pixels');
	handles.panel = panel;
	set(handles.fig, 'Units', 'pixels');
	%set(handles.fig, 'ResizeFcn', @(src,event) resize_callback(self.robot.model, handles));
	
	%% gui components
	%Exit button and callback
    uicontrol(panel, 'Style', 'pushbutton', ...
        'Units', 'normalized', ...
        'Position', [0.83 0.05 0.15 0.15], ...
        'FontUnits', 'normalized', ...
        'FontSize', 0.6, ...
        'CallBack', @(src,event) quit_callback(self.robot.model, handles), ...
        'BackgroundColor', 'white', ...
        'ForegroundColor', 'red', ...
        'String', 'X');

    %coffee1 button
	handles.order1 = uicontrol(panel, 'Style', 'pushbutton', ...
		'Units', 'normalized', ...
		'String', 'Physics', ...
		'Position', [0.12 0.85 0.6 0.15]);

    %coffee2 button
	handles.order2 = uicontrol(panel, 'Style', 'pushbutton', ...
		'Units', 'normalized', ...
		'String', 'Chemistry', ...
		'Position', [0.12 0.65 0.6 0.15]);

	%coffee3 button
	handles.order3 = uicontrol(panel, 'Style', 'pushbutton', ...
		'Units', 'normalized', ...
		'String', 'Maths', ...
		'Position', [0.12 0.45 0.6 0.15]);
    	
    %continue button
	handles.continue = uicontrol(panel, 'Style', 'pushbutton', ...
		'Units', 'normalized', ...
		'String', 'Continue', ...
		'Position', [0.12, 0.25, 0.6, 0.15]);
	
	%Emergency stop button
	handles.emergencyStop = uicontrol(panel, 'Style', 'togglebutton', ...
		'Units', 'normalized', ...
		'String', 'E-STOP', ...
        'ForegroundColor', 'red', ...
		'FontSize', 8, ...
		'Position', [0.12 0.05 0.6 0.15]);

		
	%% Callbacks
	% callback
    set(handles.order1, ...
		'Interruptible', 'on', ...
		'Callback', @(src, event)order1_callback(src, self, handles));

    set(handles.order2, ...
		'Interruptible', 'on', ...
		'Callback', @(src, event)order2_callback(src, self, handles));
	
    set(handles.order3, ...
		'Interruptible', 'on', ...
		'Callback', @(src, event)order3_callback(src, self, handles));

	set(handles.emergencyStop, ...
		'Interruptible', 'off', ...
		'Callback', @(src, event)emergencyStop_callback(src, self, handles));

	set(handles.continue, ...
		'Interruptible', 'off', ...
		'Callback', @(src, event)continue_callback(src, self, handles));
	
end

%% Callback Functions
function order1_callback(src, self, handles)
	self.orderReady = 1;
    self.startRobot = 1;
    disp("Setting up workspace for Physics...");
end

function order2_callback(src, self, handles)
	self.orderReady = 2;
    self.startRobot = 1;
    disp("Setting up workspace for Chemistry...");
end

function order3_callback(src, self, handles)
	self.orderReady = 3;
    self.startRobot = 1;
    disp("Setting up workspace for Maths...");
end

function emergencyStop_callback(src, self, handles)

button_state = get(handles.emergencyStop,'Value');

if(button_state == 1) %if estop is currently "released"
    set(handles.emergencyStop,'string','Release','foregroundcolor','blue','BackgroundColor', '#B2BEB5')
	self.estop = 1; %turn on estop
    self.robotRunning = 0; %set continue off 
    disp("WARNING! EMERGENCY STOP HAS BEEN ENGAGED!");
else
%     set(handles.emergencyStop,'string','release','foregroundcolor','blue','BackgroundColor', 'grey')
   set(handles.emergencyStop,'String','E-STOP','ForegroundColor','red', 'BackgroundColor', 'white')
   self.estop = 0; %turn off/"release" estop
   disp("Emergency Stop has been released");
end
end

function continue_callback(src, self, handles)
    if self.estop == 0 %if the estop has been released
	self.robotRunning = 1; %tell robot to continue to run
    disp("Continue setting up...");
    end
end

%callback to quit gui
function quit_callback(robot, handles)
    set(handles.fig, 'ResizeFcn', '');
    delete(handles.panel);
    set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
end

%callback to resize
% function resize_callback(handles)
%    fig = gcbo;
%    fs = get(fig, 'Position');
%    ps = get(handles.panel, 'Position');
%    set(handles.curax, 'Units', 'normalized', ...
% 	   'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
%    set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
% end