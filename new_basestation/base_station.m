%==========================================================================
% INITIALIZATION CODE
%==========================================================================


function varargout = base_station(varargin)
% BASE_STATION MATLAB code for base_station.fig
%      BASE_STATION, by itself, creates a new BASE_STATION or raises the existing
%      singleton*.
%
%      H = BASE_STATION returns the handle to a new BASE_STATION or the handle to
%      the existing singleton*.
%
%      BASE_STATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in BASE_STATION.M with the given input arguments.
%
%      BASE_STATION('Property','Value',...) creates a new BASE_STATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before base_station_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to base_station_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help base_station

% Last Modified by GUIDE v2.5 04-Mar-2018 12:55:46

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @base_station_OpeningFcn, ...
                   'gui_OutputFcn',  @base_station_OutputFcn, ...
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
end
% End initialization code - DO NOT EDIT

%==========================================================================
% VARIABLES AND CONSTANTS
%==========================================================================


% MATLAB sucks.
% MATLAB is not a real programming language.
function initVarsAndConstants(handles1)
    global handles;
    global boatMode;
    global boatPos;
    global boatTransform;
    global windVec;
    global sailDir;
    global tailDir;
    
    global wayPoints;
    global buoyPoints;
    
    global DIR_LINE_LENGTH;
    global WIND_VEC_SCALE;
    
    handles = handles1;
    
    boatMode = 0;
    boatPos = [30 30];
    boatTransform = [1 0 0; 0 1 0; 30 30 1];
    windVec = [10 30];
    sailDir = [1 -.4];
    tailDir = [-.6 1];
    wayPoints = [325 58; 56 233];
    buoyPoints = [78 97; 253 75];
    
    DIR_LINE_LENGTH = 40;
    WIND_VEC_SCALE = 2;
end


%==========================================================================
% FUNCTIONS
%==========================================================================


function drawTextureAt(path, transform)
    [im, map, alpha] = imread(path);
    %imshow(A, 'Parent', handles.CanvasAxes);
    
    [sizeX, sizeY, ~] = size(im);
    
    tx = -sizeX/2 - .5;
    ty = -sizeY/2 - .5;
        
    firstTranslate = [1 0 0; 0 1 0; ty tx 1];  % ty and tx are flipped in matrix dims
    %rotate = [1 0 0; 0 1 0; 0 0 1];
    %lastTranslate = [1 0 0; 0 1 0; 30 30 1];
    tform = affine2d(firstTranslate * transform);
    [im, imRef] = imwarp(im, tform);
    alpha = imwarp(alpha, tform);
        
    image = imshow(im, imRef);
    image.AlphaData = alpha;
end

function drawVector(origin, vec, flags)
    x1 = origin(1);
    y1 = origin(2);
    plot([x1 x1+vec(1)], [y1 y1+vec(2)], flags);
end


%==========================================================================
% EVENT HANDLERS
%==========================================================================

% --- Executes just before base_station is made visible.
function base_station_OpeningFcn(hObject, eventdata, handles, varargin)
    % This function has no output args, see OutputFcn.
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)
    % varargin   command line arguments to base_station (see VARARGIN)

    % Choose default command line output for base_station
    handles.output = hObject;

    % Update handles structure
    guidata(hObject, handles);

    % UIWAIT makes base_station wait for user response (see UIRESUME)
    % uiwait(handles.figure1);
    
    initVarsAndConstants(handles);

end


% --- Outputs from this function are returned to the command line.
function varargout = base_station_OutputFcn(hObject, eventdata, handles) 
    % varargout  cell array for returning output args (see VARARGOUT);
    % hObject    handle to figure
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Get default command line output from handles structure
    varargout{1} = handles.output;
end


% --- Executes on selection change in ModeSelect.
function ModeSelect_Callback(hObject, eventdata, handles)
% hObject    handle to ModeSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns ModeSelect contents as cell array
%        contents{get(hObject,'Value')} returns selected item from ModeSelect
end


% --- Executes during object creation, after setting all properties.
function ModeSelect_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to ModeSelect (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: popupmenu controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end



function WaypointIn_Callback(hObject, eventdata, handles)
    % hObject    handle to WaypointIn (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    structure with handles and user data (see GUIDATA)

    % Hints: get(hObject,'String') returns contents of WaypointIn as text
    %        str2double(get(hObject,'String')) returns contents of WaypointIn as a double
end


% --- Executes during object creation, after setting all properties.
function WaypointIn_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to WaypointIn (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: edit controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on selection change in WaypointList.
function WaypointList_Callback(hObject, eventdata, handles)
% hObject    handle to WaypointList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns WaypointList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from WaypointList
end


% --- Executes during object creation, after setting all properties.
function WaypointList_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to WaypointList (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: listbox controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on button press in WaypointAdd.
function WaypointAdd_Callback(hObject, eventdata, handles)
% hObject    handle to WaypointAdd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on button press in WaypointUp.
function WaypointUp_Callback(hObject, eventdata, handles)
% hObject    handle to WaypointUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on button press in WaypointDown.
function WaypointDown_Callback(hObject, eventdata, handles)
% hObject    handle to WaypointDown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on button press in BuoyDown.
function BuoyDown_Callback(hObject, eventdata, handles)
% hObject    handle to BuoyDown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on button press in BuoyUp.
function BuoyUp_Callback(hObject, eventdata, handles)
% hObject    handle to BuoyUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on button press in BuoyAdd.
function BuoyAdd_Callback(hObject, eventdata, handles)
% hObject    handle to BuoyAdd (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end


% --- Executes on selection change in BuoyList.
function BuoyList_Callback(hObject, eventdata, handles)
% hObject    handle to BuoyList (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns BuoyList contents as cell array
%        contents{get(hObject,'Value')} returns selected item from BuoyList
end


% --- Executes during object creation, after setting all properties.
function BuoyList_CreateFcn(hObject, eventdata, handles)
    % hObject    handle to BuoyList (see GCBO)
    % eventdata  reserved - to be defined in a future version of MATLAB
    % handles    empty - handles not created until after all CreateFcns called

    % Hint: listbox controls usually have a white background on Windows.
    %       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end



function BuoyIn_Callback(hObject, eventdata, handles)
% hObject    handle to BuoyIn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BuoyIn as text
%        str2double(get(hObject,'String')) returns contents of BuoyIn as a double
end


% --- Executes during object creation, after setting all properties.
function BuoyIn_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BuoyIn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
    if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
        set(hObject,'BackgroundColor','white');
    end
end


% --- Executes on button press in DeleteButton.
function DeleteButton_Callback(hObject, eventdata, handles)
% hObject    handle to DeleteButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    updateCanvas();
    updateWaypointList();
    updateBuoyList();
end


%==========================================================================
% PUBLIC FUNCTIONS
%==========================================================================


function rotate = createRotZ(angle)
    rotate = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
end

function translate = createTranslation(pos)
    translate = [1 0 0; 0 1 0; pos(1) pos(2) 1];
end

function updateFromData(data)
    rotation = createRotZ(data.boat_heading);
    translate = createTranslation(data.position);
    boatTransform = rotation * translate;
end

function updateCanvas()
    global boatMode;
    global boatPos;
    global boatTransform;
    global windVec;
    global sailDir;
    global tailDir;
    
    global wayPoints;
    global buoyPoints;
    
    global DIR_LINE_LENGTH;
    global WIND_VEC_SCALE;
    
    drawTextureAt('Boat.png', boatTransform);
    drawVector(boatPos, windVec * WIND_VEC_SCALE, 'k');
    drawVector(boatPos, sailDir * DIR_LINE_LENGTH, '--r');
    drawVector(boatPos, tailDir * DIR_LINE_LENGTH, '--b');
    
    for row = 1:size(wayPoints, 1)
        point = wayPoints(row, :);
        drawTextureAt('Waypoint.png', createTranslation(point));
    end
    
    for row = 1:size(buoyPoints, 1)
        point = buoyPoints(row, :);
        drawTextureAt('Buoy.png', createTranslation(point));
    end
end

function str = formatPoint(pt)
    str = sprintf('(%.0f, %.0f)', pt(1), pt(2));
end

function updateWaypointList()
    global handles;
    global wayPoints;
    
    numPoints = size(wayPoints, 1);
    stringVec = [];

    for row = 1:numPoints
        stringVec = [stringVec; formatPoint(wayPoints(row, :))];
    end
    
    handles.WaypointList.String = stringVec;
end

function updateBuoyList()
    global handles;
    global buoyPoints;
    
    numPoints = size(buoyPoints, 1);
    stringVec = [];

    for row = 1:numPoints
        disp(buoyPoints(row, :));
        stringVec = [stringVec; formatPoint(buoyPoints(row, :))];
    end
    
    handles.BuoyList.String = stringVec;
end
