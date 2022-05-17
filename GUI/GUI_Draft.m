function varargout = GUI_Draft(varargin)
%     clc;
    % Begin initialization code - DO NOT EDIT
    gui_Singleton = 1;
    gui_State = struct('gui_Name', mfilename, ...
        'gui_Singleton', gui_Singleton, ...
        'gui_OpeningFcn', @GUI_Draft_OpeningFcn, ...
        'gui_OutputFcn', @GUI_Draft_OutputFcn, ...
        'gui_LayoutFcn', [], ...
        'gui_Callback', []);

    if nargin && ischar(varargin{1})
        gui_State.gui_Callback = str2func(varargin{1});
    end

    if nargout
        [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
    else
        gui_mainfcn(gui_State, varargin{:});
    end

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

    % This sets up the initial plot - only do when we are invisible so window can get raised using GUI_Draft.
    if strcmp(get(hObject, 'Visible'), 'off')
        %plot(rand(5));
    end

    [x, map] = imread('estop.jpg');
    I2 = imresize(x, [90 90]);
    set(handles.e_stop, 'cdata', I2);
end

% --- Outputs from this function are returned to the command line.
function varargout = GUI_Draft_OutputFcn(hObject, eventdata, handles)
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end

%todo --------------- Workspace Setup
% --- Executes on button press in load_workspace.
function load_workspace_Callback(hObject, eventdata, handles)
    % hObject    handle to load_workspace (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % Add ground image and set the size of the world - cla
    %axes(handles.axes1);
    figure (1); % Spawns the robot environment in a separate figure
    hold on

    worldCoords = 0.6;
    axis([-worldCoords worldCoords -worldCoords worldCoords 0 0.7]); %minX maxX minY maxY minZ maxZ
    surf([-worldCoords, -worldCoords; worldCoords, worldCoords], [-worldCoords, worldCoords; -worldCoords, worldCoords], [0, 0; 0, 0], 'CData', imread('marble.jpg'), 'FaceColor', 'texturemap');

    %Set robot base locations
    heightDobot = 0;
    baseDobot = [0, 0, heightDobot];

    %Add Safety Equipment
    PlaceObject('EmergencyButton.ply', [0.5, 0.5, heightDobot]);
    PlaceObject('Basket.ply', [0.25, 0.17, heightDobot]);
    LightCurtain(heightDobot);

    robot = Dobot(transl(baseDobot));
    robot.model.animate(deg2rad([-15 40 60 12.5 0]));

    imageBlocks = load("image.mat");
    coords = cam2Robot(imageBlocks.image);

    %? Adding Objects
    figure (1); % Spawns the robot environment in a separate figure
    data = guidata(hObject); % must stay above any changes to data
    locationStrawberry = [0.22, -0.2, heightDobot];
    data.strawberry = Strawberry(locationStrawberry);
    data.countStrawberry = 0;

    locationGrape = [0.05, -0.18, heightDobot];
    data.grape = Grape(locationGrape);
    data.countGrape = 0;

    locationLego = [0.10, -0.25, heightDobot];
    data.bluelego = Lego(locationLego,'blue');

    locationLego = [0.274, -0.007, heightDobot];
    data.greenlego = Lego(locationLego,'green');

    locationLego = [0.16, -0.147, heightDobot];
    data.yellowlego = Lego(locationLego,'yellow');

    locationLego = [0.215, -0.058, heightDobot];
    data.orangelego = Lego(locationLego,'orange');
    data.countLego = 0;

    data.model = robot.model;
    data.robot = robot;
    data.estop_count = 0;
    data.imageEstopOn = imread("estopON.jpg");
    data.stop = false;
    data.pcPoints = [10 10 10];
    data.objectPos = coords;
    % data.fig = fig;
    %data.jointLimits = jointLimits;

    guidata(hObject, data);
end

% --------------------------------------------------------------------
function FileMenu_Callback(hObject, eventdata, handles)
end

% --------------------------------------------------------------------
function OpenMenuItem_Callback(hObject, eventdata, handles)
    file = uigetfile('*.fig');

    if ~isequal(file, 0)
        open(file);
    end

end

% --------------------------------------------------------------------
function PrintMenuItem_Callback(hObject, eventdata, handles)
    printdlg(handles.figure1)
end

% --------------------------------------------------------------------
function CloseMenuItem_Callback(hObject, eventdata, handles)
    selection = questdlg(['Close ' get(handles.figure1, 'Name') '?'], ...
        ['Close ' get(handles.figure1, 'Name') '...'], ...
        'Yes', 'No', 'Yes');

    if strcmp(selection, 'No')
        return;
    end

    delete(handles.figure1)
end

%todo -------------------------joint sliders
% --- Executes on slider movement.
function Q1slider_Callback(hObject, eventdata, handles)
    % Hints: get(hObject,'Value') returns position of slider
    %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    if (handles.stop == false)
        val = get(hObject, 'Value');
        val = round(val);
        q = handles.model.getpos();
        q(1) = deg2rad(val);
        collision = willCollide(handles.robot, q, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(q);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes during object creation, after setting all properties.
function Q1slider_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'Max', 130, 'Min', -130);
    set(hObject, 'SliderStep', [1/259 1]);
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject, 'BackgroundColor'), get(0, 'defaultUicontrolBackgroundColor'))
        set(hObject, 'BackgroundColor', [.9 .9 .9]);
    end

end

% --- Executes on slider movement.
function Q2slider_Callback(hObject, eventdata, handles)
    % Hints: get(hObject,'Value') returns position of slider
    %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    if (handles.stop == false)
        val = get(hObject, 'Value');
        val = round(val);
        q = handles.model.getpos();
        q(2) = deg2rad(val);
        q(4) =- (pi / 2 - q(2) - q(3));
        collision = willCollide(handles.robot, q, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(q);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes during object creation, after setting all properties.
function Q2slider_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'Max', 80, 'Min', 5);
    set(hObject, 'SliderStep', [1/74 1]);
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject, 'BackgroundColor'), get(0, 'defaultUicontrolBackgroundColor'))
        set(hObject, 'BackgroundColor', [.9 .9 .9]);
    end

end

% --- Executes on slider movement.
function Q3slider_Callback(hObject, eventdata, handles)
    % Hints: get(hObject,'Value') returns position of slider
    %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    if (handles.stop == false)
        val = get(hObject, 'Value');
        val = round(val);
        q = handles.model.getpos();
        q(3) = deg2rad(val);
        q(4) =- (pi / 2 - q(2) - q(3));
        collision = willCollide(handles.robot, q, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(q);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes during object creation, after setting all properties.
function Q3slider_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'Max', 85, 'Min', -5);
    set(hObject, 'SliderStep', [1/89 1]);
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject, 'BackgroundColor'), get(0, 'defaultUicontrolBackgroundColor'))
        set(hObject, 'BackgroundColor', [.9 .9 .9]);
    end

end

% --- Executes on slider movement.
function Q5slider_Callback(hObject, eventdata, handles)
    % Hints: get(hObject,'Value') returns position of slider
    %        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
    if (handles.stop == false)
        val = get(hObject, 'Value');
        val = round(val);
        q = handles.model.getpos();
        q(5) = deg2rad(val);
        collision = willCollide(handles.robot, q, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(q);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes during object creation, after setting all properties.
function Q5slider_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'Max', 85, 'Min', -185);
    set(hObject, 'SliderStep', [1/169 1]);
    % Hint: slider controls usually have a light gray background.
    if isequal(get(hObject, 'BackgroundColor'), get(0, 'defaultUicontrolBackgroundColor'))
        set(hObject, 'BackgroundColor', [.9 .9 .9]);
    end

end

%-----------------------------------------------------------------------------------------------%
%todo----------------------------XYZ Buttons-----------------------------------------------------%
%vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv%
% --- Executes on button press in plusX.
function plusX_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        q = handles.model.getpos();
        startPoint = handles.model.fkine(q);
        startPoint = startPoint(1:3, 4);
        endPoint = startPoint;
        endPoint(1) = endPoint(1) + 0.01;
        newQ = XYZtoQ(endPoint);
        collision = willCollide(handles.robot, newQ, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(newQ);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes on button press in minusX.
function minusX_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        q = handles.model.getpos();
        startPoint = handles.model.fkine(q);
        startPoint = startPoint(1:3, 4);
        endPoint = startPoint;
        endPoint(1) = endPoint(1) - 0.01;
        newQ = XYZtoQ(endPoint);
        collision = willCollide(handles.robot, newQ, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(newQ);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes on button press in minusY.
function minusY_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        q = handles.model.getpos();
        startPoint = handles.model.fkine(q);
        startPoint = startPoint(1:3, 4);
        endPoint = startPoint;
        endPoint(2) = endPoint(2) - 0.01;
        newQ = XYZtoQ(endPoint);
        collision = willCollide(handles.robot, newQ, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(newQ);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes on button press in plusY.
function plusY_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        q = handles.model.getpos();
        startPoint = handles.model.fkine(q);
        startPoint = startPoint(1:3, 4);
        endPoint = startPoint;
        endPoint(2) = endPoint(2) + 0.01;
        newQ = XYZtoQ(endPoint);
        collision = willCollide(handles.robot, newQ, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(newQ);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes on button press in minusZ.
function minusZ_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        q = handles.model.getpos();
        startPoint = handles.model.fkine(q);
        startPoint = startPoint(1:3, 4);
        endPoint = startPoint;
        endPoint(3) = endPoint(3) - 0.01;
        newQ = XYZtoQ(endPoint);
        collision = willCollide(handles.robot, newQ, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(newQ);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% --- Executes on button press in plusZ.
function plusZ_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        q = handles.model.getpos();
        startPoint = handles.model.fkine(q);
        startPoint = startPoint(1:3, 4);
        endPoint = startPoint;
        endPoint(3) = endPoint(3) + 0.01;
        newQ = XYZtoQ(endPoint);
        collision = willCollide(handles.robot, newQ, handles.pcPoints);
        intruder = lightCurtainCheck(handles.pcPoints);

        if collision == false && intruder == false
            handles.model.animate(newQ);
        elseif collision == true
            cprintf('red', 'Possible collision detected! Aborting move!\n');
            beep
        elseif intruder == true
            cprintf([1, 0.5, 0], 'Object in workspace! Halting operation!\n');
            beep
        end

    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

%todo ------------------------Rotate Robot
% --- Executes on button press in RotateRobot.
function RotateRobot_Callback(hObject, eventdata, handles)

    if (handles.stop == false)
        handles.robot.Spin();
    else
        msgbox("Error! Either E-stop engaged or Resume not selected!", "Error", "error");
        beep;
    end

end

% UIWAIT makes GUI_Draft wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%todo------------------------------------E-STOP-----------------------------------------------------%
%vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv%
% --- Executes on button press in e_stop.
function e_stop_Callback(hObject, eventdata, handles)
    % f = figure(f);
    % c = uicontrol('String','Continue','Callback','uiresume(f)');

    % disp('Program execution has resumed');
    if handles.estop_count == 0 || handles.estop_count == 2
        handles.estop_count = 1;
        handles.stop = true;
        beep;
        [x, map] = imread('estopOn.jpg');
        I2 = imresize(x, [90 90]);
        set(handles.e_stop, 'cdata', I2);
        guidata(hObject, handles);
        uiwait();
        handles.estop_count = 0;
    end

    if handles.estop_count == 1

        if handles.estop_count == 1
            msgbox("        !E-STOP RELEASED!", "Caution", "warn", 'modal');
            beep;
        end

        handles.estop_count = 2;
        [x, map] = imread('estop.jpg');
        I = imresize(x, [90 90]);
        set(handles.e_stop, 'cdata', I);

        guidata(hObject, handles);
    end

end

%todo ------------ Resume Motion
% --- Executes on button press in resume_function.
function resume_function_Callback(hObject, eventdata, handles)

    if handles.estop_count == 1
        guidata(hObject, handles);
        msgbox("Please release E-stop before continuing", "Notice", "warn");
        beep;
        %     handles.estop_count = 2;
    end

    if handles.estop_count == 2
        disp('Resuming');
        uiresume();
        handles.estop_count = 0;
        handles.stop = false;
        guidata(hObject, handles);
    end

end

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject, 'BackgroundColor'), get(0, 'defaultUicontrolBackgroundColor'))
        set(hObject, 'BackgroundColor', 'white');
    end

end

%todo ------------------- Reset Robot configuration
%  --- Executes on button press in ResetRobot.
function ResetRobot_Callback(hObject, eventdata, handles)
    handles.model.animate(deg2rad([0 40 60 10 0]));
end

%todo------------------------------------Item Selection-----------------------------------------------------%
% --- Executes on selection change in itemSelector.
function itemSelector_Callback(hObject, eventdata, handles)
    % Hints: contents = cellstr(get(hObject,'String')) returns itemSelector contents as cell array
    %        contents{get(hObject,'Value')} returns selected item from itemSelector
    contents = cellstr(get(hObject, 'String'));
    selection = contents{get(hObject, 'Value')};
    handles.selection = selection; %Just sets global variable to selction got Get button to read
    guidata(hObject, handles);
end

% --- Executes during object creation, after setting all properties.
function itemSelector_CreateFcn(hObject, eventdata, handles)
    set(hObject, 'String', {'Strawberry', 'Grape', 'Lego','Warm Legos'}); %change these values to names of whatever is being picked up
    handles.selection = 'Strawberry'; % default selection
    guidata(hObject, handles);

    if ispc && isequal(get(hObject, 'BackgroundColor'), get(0, 'defaultUicontrolBackgroundColor'))
        set(hObject, 'BackgroundColor', 'white');
    end

end

% --- Executes on button press in getSelection.
function getSelection_Callback(hObject, eventdata, handles)
    % hObject    handle to getSelection (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    selection = handles.selection; %gets whatever is selected in drop down box as string
    disp(selection);
    figure(1)
%     RotateRobot(handles.robot, -30);
    endPoint = handles.objectPos.basket;
    switch selection
        case 'Strawberry'
            if  handles.countStrawberry == 0
                midPoint = handles.strawberry.location;
                midPoint(3) = midPoint(3) + 0.02;
                endPoint(3) = midPoint(3);
                endPoint(2) = endPoint(2) - 0.02;                           %adjust to side of basket
                getThing(hObject, eventdata, handles,midPoint,endPoint,handles.strawberry,'strawberry');
                handles.countStrawberry = handles.countStrawberry + 1;
            else 
                disp("Item already picked up. Choose a different one");
            end  
            RotateRobot(handles.robot, -30);
        case 'Grape'
            if handles.countGrape == 0
                midPoint = handles.grape.location;
                midPoint(3) = midPoint(3) + 0.02;
                endPoint(3) = midPoint(3);
                endPoint(2) = endPoint(2) - 0.02;                           %adjust to side of basket
                getThing(hObject, eventdata, handles,midPoint,endPoint,handles.grape,'grape');
                handles.countGrape = handles.countGrape + 1;
            else
                disp("Item already picked up. Choose a different one");
            end
            RotateRobot(handles.robot, -30);
        case 'Lego'
            if handles.countLego == 0
                midPoint = handles.bluelego.location;
                midPoint(3) = midPoint(3) + 0.02;
                endPoint(3) = midPoint(3);
                endPoint(2) = endPoint(2) - 0.02;                           %adjust to side of basket
                getThing(hObject, eventdata, handles,midPoint,endPoint,handles.bluelego,'lego');
                handles.countLego = handles.countLego + 1;
            else
                disp("Item already picked up. Choose a different one");
            end
            RotateRobot(handles.robot, -30);
        case 'Warm Legos'
                midPoint = handles.objectPos.orange;                       %get location of orange block from image scan
                midPoint(3) = midPoint(3) + -midPoint(3) + 0.02;                 %ignore depth value from camera, set to 0 -0.0655 + -(-0.0665) = 0 
                endPoint(3) = midPoint(3);
                endPoint(2) = endPoint(2) - 0.02;                          %adjust to side of basket
                getThing(hObject, eventdata, handles,midPoint,endPoint,handles.orangelego,'strawberry');
                RotateRobot(handles.robot, -30);
                midPoint = handles.objectPos.yellow;                       %get location of yellow block from image scan
                midPoint(3) = midPoint(3) + -midPoint(3) + 0.02;                  %ignore depth value from camera, set to 0 -0.0655 + -(-0.0665) = 0
                endPoint(3) = midPoint(3);
                endPoint(2) = endPoint(2) - 0.02;  %adjust to side of basket
                getThing(hObject, eventdata, handles,midPoint,endPoint,handles.yellowlego,'strawberry');
                RotateRobot(handles.robot, -30);
    end
    guidata(hObject, handles); %update object counts

end

%todo -----------------------------------Collision & Lightcurtain Demoing---------------------------------------------------%
% --- Executes on button press in spawnObstacle.
function spawnObstacle_Callback(hObject, eventdata, handles)
    ptCloud = pcread('square.ply');
    cubePoints = ptCloud.Location;
    move = [0.2, 0.1, 0.2];
    cubePoints = cubePoints + repmat(move, size(cubePoints, 1), 1);
    figure (1)
    hold on
    mesh_h = PlaceObject('square.ply', move);
    handles.cube = mesh_h;
    handles.pcPoints = [handles.pcPoints; cubePoints];
    guidata(hObject, handles);
    % cube_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3),'b.');
end

% --- Executes on button press in spawnArm.
function spawnArm_Callback(hObject, eventdata, handles)
    ptCloud = pcread('arm.ply');
    armPoints = ptCloud.Location;
    figure (1)
    hold on
    mesh_h = PlaceObject('arm.ply', [0.9, 0, 0.1]);
    vertices = get(mesh_h, 'Vertices');
    steps = 50;
    t = transl(0.9, 0, 0.1);
    tr = t;

    for i = 1:50
        tr(1, 4) = tr(1, 4) - (0.001 * i);
        move = tr(1:3, 4)';
        transformedVertices = [vertices, ones(size(vertices, 1), 1)] * tr';
        set(mesh_h, 'Vertices', transformedVertices(:, 1:3));
        armPoints = repmat(move, size(armPoints, 1), 1);
        drawnow();
        pause(0.01);
        
    end
    handles.pcPoints = armPoints;
    guidata(hObject, handles);
    handles.arm = mesh_h;
    guidata(hObject, handles);
end

% --- Executes on button press in clearObjs.
function clearObjs_Callback(hObject, eventdata, handles)
    figure (1)
    try delete(handles.arm); end;
    try delete(handles.cube); end;
    handles.pcPoints = [10, 10, 10];
    guidata(hObject, handles);
end
