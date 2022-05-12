function varargout = GUI_Draft(varargin)
% GUI_DRAFT MATLAB code for GUI_Draft.fig
%      GUI_DRAFT, by itself, creates a new GUI_DRAFT or raises the existing
%      singleton*.
%
%      H = GUI_DRAFT returns the handle to a new GUI_DRAFT or the handle to
%      the existing singleton*.
%
%      GUI_DRAFT('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_DRAFT.M with the given input arguments.
%
%      GUI_DRAFT('Property','Value',...) creates a new GUI_DRAFT or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_Draft_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_Draft_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI_Draft

% Last Modified by GUIDE v2.5 12-May-2022 14:13:41

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_Draft_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_Draft_OutputFcn, ...
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

% --- Executes just before GUI_Draft is made visible.
function GUI_Draft_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI_Draft (see VARARGIN)

% Choose default command line output for GUI_Draft
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% This sets up the initial plot - only do when we are invisible
% so window can get raised using GUI_Draft.
if strcmp(get(hObject,'Visible'),'off')
    %plot(rand(5));
end
[x,map]=imread('estop.jpg');
I2=imresize(x, [42 113]);
set(handles.e_stop,'cdata',I2);


% UIWAIT makes GUI_Draft wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_Draft_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in pushbutton15.
function load_workspace_Callback(hObject, eventdata, handles)
% hObject    handle to load_workspace (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Add ground image and set the size of the world
cla
fig = figure (1)
%axes(handles.axes1);
hold on

worldCoords = 0.6;
axis([-worldCoords worldCoords -worldCoords worldCoords 0.7 1.3]); %minX maxX minY maxY minZ maxZ
surf([-worldCoords, -worldCoords; worldCoords, worldCoords], [-worldCoords, worldCoords; -worldCoords, worldCoords], [0, 0; 0, 0], 'CData', imread('marble.jpg'), 'FaceColor', 'texturemap');

% Adding objects to scene
tableHeight = 0.711547;
tableLength = 1.181 * 2;
tableWidth = 0.7387 * 2;
tablegapX = 0.005;
PlaceObject('Table.ply', [0, 0, 0]);

%Set robot base locations
baseDobot = [0, 0, tableHeight];

%Add Safety Equipment
PlaceObject('EmergencyButton.ply', [0.5, 0.5, tableHeight]);

% Plot lines between lightcurtains
LightCurtain(tableHeight);

PlaceObject('Basket.ply', [0.25, 0.025, tableHeight]);

%robot = Dobot;
robot = Dobot(transl(baseDobot));
robot.model.animate(deg2rad([-15 40 60 12.5 0]));


data = guidata(hObject);
data.model = robot.model;
data.robot = robot;
% data.fig = fig;
data.stop = false;

guidata(hObject,data);



% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
% hObject    handle to FileMenu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to OpenMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
file = uigetfile('*.fig');
if ~isequal(file, 0)
    open(file);
end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to PrintMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
printdlg(handles.figure1)

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
% hObject    handle to CloseMenuItem (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
selection = questdlg(['Close ' get(handles.figure1,'Name') '?'],...
                     ['Close ' get(handles.figure1,'Name') '...'],...
                     'Yes','No','Yes');
if strcmp(selection,'No')
    return;
end

delete(handles.figure1)


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = get(hObject,'String') returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
     set(hObject,'BackgroundColor','white');
end

set(hObject, 'String', {'plot(rand(5))', 'plot(sin(1:0.01:25))', 'bar(1:.5:10)', 'plot(membrane)', 'surf(peaks)'});


% --- Executes on button press in minusq1.
function minusq1_Callback(hObject, eventdata, handles)
% hObject    handle to minusq1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

q1 = handles.model.getpos;
qNext = q1-deg2rad([5 0 0 0 0]);
handles.model.animate(qNext);


% --- Executes on button press in plusq1.
function plusq1_Callback(hObject, eventdata, handles)
% hObject    handle to plusq1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

q1 = handles.model.getpos;
qNext = q1+deg2rad([5 0 0 0 0]);
handles.model.animate(qNext);

% --- Executes on button press in minusq2.
function minusq2_Callback(hObject, eventdata, handles)
% hObject    handle to minusq2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q2 = handles.model.getpos;
qNext = q2-deg2rad([0 5 0 0 0]);
qNext(4) = -(pi/2 - qNext(2) - qNext(3));
handles.model.animate(qNext);


% --- Executes on button press in plusq2.
function plusq2_Callback(hObject, eventdata, handles)
% hObject    handle to plusq2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q2 = handles.model.getpos;
qNext = q2+deg2rad([0 5 0 0 0]);
qNext(4) = -(pi/2 - qNext(2) - qNext(3));
handles.model.animate(qNext);


% --- Executes on button press in minusq3.
function minusq3_Callback(hObject, eventdata, handles)
% hObject    handle to minusq3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q3 = handles.model.getpos;
lims = handles.robot.jointLimits;
[~, index] = min(abs(lims(:, 1) - q3(1, 2)));
[~, index2] = min(abs(lims(:, 3) - q3(1, 2)));
qlim(1, 1) = lims(index, 2);
qlim(1, 2) = lims(index2, 4);
qNext = q3-deg2rad([0 0 5 0 0]);
qNext(4) = -(pi/2 - qNext(2) - qNext(3));
handles.model.animate(qNext);


% --- Executes on button press in plusq3.
function plusq3_Callback(hObject, eventdata, handles)
% hObject    handle to plusq3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q3 = handles.model.getpos;
lims = handles.robot.jointLimits;
[~, index] = min(abs(lims(:, 1) - q3(1, 2)));
[~, index2] = min(abs(lims(:, 3) - q3(1, 2)));
qlim(1, 1) = lims(index, 2);
qlim(1, 2) = lims(index2, 4);
qNext = q3+deg2rad([0 0 5 0 0]);
qNext(4) = -(pi/2 - qNext(2) - qNext(3));
handles.model.animate(qNext);


% --- Executes on button press in minusq5.
function minusq5_Callback(hObject, eventdata, handles)
% hObject    handle to minusq5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q5 = handles.model.getpos;
qNext = q5-deg2rad([0 0 0 0 5]);
handles.model.animate(qNext);


% --- Executes on button press in plusq5.
function plusq5_Callback(hObject, eventdata, handles)
% hObject    handle to plusq5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q5 = handles.model.getpos;
qNext = q5+deg2rad([0 0 0 0 5]);
handles.model.animate(qNext);

% --- Executes on button press in plusX.
function plusX_Callback(hObject, eventdata, handles)
% hObject    handle to plusX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos();
startPoint = handles.model.fkine(q);
startPoint = startPoint(1:3,4);
endPoint = startPoint;
endPoint(1) = endPoint(1) + 0.01;
newQ = XYZtoQ(endPoint);

handles.model.animate(newQ);


% --- Executes on button press in minusX.
function minusX_Callback(hObject, eventdata, handles)
% hObject    handle to minusX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

% --- Executes on button press in minusY.
function minusY_Callback(hObject, eventdata, handles)
% hObject    handle to minusY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in plusY.
function plusY_Callback(hObject, eventdata, handles)
% hObject    handle to plusY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in minusZ.
function minusZ_Callback(hObject, eventdata, handles)
% hObject    handle to minusZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in plusZ.
function plusZ_Callback(hObject, eventdata, handles)
% hObject    handle to plusZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)






% --- Executes on button press in pushbutton25.
function pushbutton25_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
