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

% Last Modified by GUIDE v2.5 05-May-2022 17:59:46

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

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

cla;
axes(handles.axes1);


name = ['Dobot', datestr(now, 'yyyymmddTHHMMSSFFF')];
base = [0, 0, 0.711547];

L(1) = Link('d', 0.138, 'a', 0, 'alpha', -pi/2, 'offset', 0,'qlim',[-135*pi/180 135*pi/180]);
L(2) = Link('d', 0, 'a', 0.135, 'alpha',0,'offset', -pi/2,'qlim',[5*pi/180 80*pi/180]);
L(3) = Link('d', 0, 'a', 0.147, 'alpha', pi, 'offset',0,'qlim',[-5*pi/180 85*pi/180]);
L(4) = Link('d', 0, 'a', 0.041, 'alpha', pi/2, 'offset', 0,'qlim',[-pi/2 pi/2]);
L(5) = Link('d', 0.09, 'a', 0, 'alpha',0, 'offset', 0,'qlim',[-85*pi/180 85*pi/180]);

model = SerialLink(L, 'name', name); %'base', base

for linkIndex = 0:model.n
    [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['DobotLink', num2str(linkIndex), '.ply'], 'tri'); %#ok<AGROW>
    model.faces{linkIndex + 1} = faceData;
    model.points{linkIndex + 1} = vertexData;
end

%Display Robot
workspace = [-1 1 -1 1 -0.3 1];
model.teach();
%model.plot3d(zeros(1,model.n), 'noarrow', 'workspace', workspace);
if isempty(findobj(get(gca, 'Children'), 'Type', 'Light'))
     camlight
end  
model.delay = 0;

% Try to correctly colour the arm (if colours are in ply file data)
for linkIndex = 0:model.n
    handles = findobj('Tag', model.name);
    h = get(handles, 'UserData');
    try 
       h.link(linkIndex + 1).Children.FaceVertexCData = [plyData{linkIndex + 1}.vertex.red ...
                                                     , plyData{linkIndex + 1}.vertex.green ...
                                                     , plyData{linkIndex + 1}.vertex.blue]/255;
       h.link(linkIndex + 1).Children.FaceColor = 'interp';
    catch ME_1
        disp(ME_1);
        continue;
    end
end

data = guidata(hObject);
data.model = model;
guidata(hObject,data);
% popup_sel_index = get(handles.popupmenu1, 'Value');
% switch popup_sel_index
%     case 1
%         plot(rand(5));
%     case 2
%         plot(sin(1:0.01:25.99));
%     case 3
%         bar(1:.5:10);
%     case 4
%         plot(membrane);
%     case 5
%         surf(peaks);
% end


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


% --- Executes on button press in plusX.
function plusX_Callback(hObject, eventdata, handles)
% hObject    handle to plusX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos();
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
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


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) + 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

q = handles.model.getpos;
tr = handles.model.fkine(q);
tr(1,4) = tr(1,4) - 0.01;
newQ = handles.model.ikcon(tr,q);
handles.model.animate(newQ);

get(hObject, 'Value');

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end
