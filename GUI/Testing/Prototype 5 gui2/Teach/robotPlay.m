function varargout = robotPlay(varargin)
% ROBOTPLAY MATLAB code for robotPlay.fig
%      ROBOTPLAY, by itself, creates a new ROBOTPLAY or raises the existing
%      singleton*.
%
%      H = ROBOTPLAY returns the handle to a new ROBOTPLAY or the handle to
%      the existing singleton*.
%
%      ROBOTPLAY('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTPLAY.M with the given input arguments.
%
%      ROBOTPLAY('Property','Value',...) creates a new ROBOTPLAY or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before robotPlay_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to robotPlay_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help robotPlay

% Last Modified by GUIDE v2.5 20-Oct-2023 23:03:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @robotPlay_OpeningFcn, ...
                   'gui_OutputFcn',  @robotPlay_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before robotPlay is made visible.
function robotPlay_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to robotPlay (see VARARGIN)

% Choose default command line output for robotPlay
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes robotPlay wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = robotPlay_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function pos_X_Callback(hObject, eventdata, handles)
% hObject    handle to pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_X as text
%        str2double(get(hObject,'String')) returns contents of pos_X as a double


% --- Executes during object creation, after setting all properties.
function pos_X_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_X (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_Y_Callback(hObject, eventdata, handles)
% hObject    handle to pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_Y as text
%        str2double(get(hObject,'String')) returns contents of pos_Y as a double


% --- Executes during object creation, after setting all properties.
function pos_Y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_Y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pos_Z_Callback(hObject, eventdata, handles)
% hObject    handle to pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pos_Z as text
%        str2double(get(hObject,'String')) returns contents of pos_Z as a double


% --- Executes during object creation, after setting all properties.
function pos_Z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pos_Z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% grabbing positions from user input
pX = str2double(handles.pos_X.String);
pY = str2double(handles.pos_Y.String);
pZ = str2double(handles.pos_Z.String);

% calling in the robot
r = Pulse75;
%robo2r = Pulse75(transl(0.5,0,0));

% location of end effector given by user
locationTR = transl(pX,pY,pZ);
%robo2locationTR = transl(pX,pY,pZ);

q = r.model.ikine(locationTR,'q0',[0 0 0 0 0 0],'mask',[1 1 1 1 0 0]);
%robo2q = robo2r.model.ikine(robo2locationTR,'q0',[0 0 0 0 0 0],'mask',[1 1 1 1 0 0]);

steps = 165;
%robo2steps = 165;

q0 = r.model.getpos();
%robo2q0 = robo2r.model.getpos();

qMatrix = jtraj(q0,q,steps);
%robo2qMatrix = jtraj(robo2q0,robo2q,robo2steps);

% animate 
for i = 1:length(qMatrix)
    r.model.animate(qMatrix(i,:)); % animate
    drawnow();
end

% for i = 1:length(robo2qMatrix)
%     robo2r.model.animate(robo2qMatrix(i,:)); % animate
%     drawnow();
% end

uiwait(helpdlg('Examine the figures, then click OK to finish.'));

%% second
% % calling in the robot
% r = UR3(transl(0.5,0,0));
% 
% % location of end effector given by user
% locationTR = transl(pX,pY,pZ);
% q = r.model.ikine(locationTR,'q0',[0 0 0 0 0 0],'mask',[1 1 1 1 0 0]);
% steps = 165;
% q0 = r.model.getpos();
% qMatrix = jtraj(q0,q,steps);
% 
% % animate 
% for i = 1:length(qMatrix)
%     r.model.animate(qMatrix(i,:)); % animate
%     drawnow();
% end
% 
% 
% uiwait(helpdlg('Examine the figures, then click OK to finish.'));
% 
