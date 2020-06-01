function varargout = main_gui(varargin)
% MAIN_GUI MATLAB code for main_gui.fig
%      MAIN_GUI, by itself, creates a new MAIN_GUI or raises the existing
%      singleton*.
%
%      H = MAIN_GUI returns the handle to a new MAIN_GUI or the handle to
%      the existing singleton*.
%
%      MAIN_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAIN_GUI.M with the given input arguments.
%
%      MAIN_GUI('Property','Value',...) creates a new MAIN_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before main_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to main_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help main_gui

% Last Modified by GUIDE v2.5 18-Jan-2020 15:47:25

%% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @main_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @main_gui_OutputFcn, ...
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


%% --- Executes just before main_gui is made visible.
function main_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to main_gui (see VARARGIN)

% Choose default command line output for main_gui
handles.output = hObject;

global model

% Read the varargin
cfg = varargin{1};
handles.UserData.cfg = cfg;

% Set default values
% ===== 3D plot =====
h3d_plot = handles.fig_main;
h3d_plot.XLabel.String = 'x [m]'; 
h3d_plot.YLabel.String = 'y [m]'; 
h3d_plot.ZLabel.String = 'z [m]';
hold(h3d_plot,'on');
grid(h3d_plot,'on');
box(h3d_plot,'on');
grid(h3d_plot,'minor');
axis(h3d_plot, [-cfg.ws(1)-0.3  cfg.ws(1)+0.3 ...
                -cfg.ws(2)-0.3  cfg.ws(2)+0.3 ...
                max(0, -cfg.ws(3)-0.4)  cfg.ws(3)+0.4]);
daspect(h3d_plot, [1 1 1]);
rotate3d(h3d_plot);

% ===== 2D plot =====
h2d_plot1 = handles.fig_topdown;
h2d_plot1.XLabel.String = 'x [m]'; 
h2d_plot1.YLabel.String = 'y [m]';
hold(h2d_plot1,'on');
grid(h2d_plot1,'on');
grid(h2d_plot1,'minor');
axis(h2d_plot1, [-cfg.ws(1)-0.4  cfg.ws(1)+0.4 ...
                 -cfg.ws(2)-0.4  cfg.ws(2)+0.4]);
daspect(h2d_plot1,[1 1 1]);

h2d_plot2 = handles.fig_side;
h2d_plot2.XLabel.String = 'x [m]'; 
h2d_plot2.YLabel.String = 'z [m]';
hold(h2d_plot2,'on');
grid(h2d_plot2,'on');
grid(h2d_plot2,'minor');
axis(h2d_plot2, [-cfg.ws(1)-0.4  cfg.ws(1)+0.4 ...
                max(0, -cfg.ws(3)-0.4)  cfg.ws(3)+0.4]);
daspect(h2d_plot2,[1 1 1]);

% ===== Set initial quad state =====
set(handles.uitable_quad_state, 'Data', zeros(7, model.nQuad), ...
    'ColumnFormat', repmat({'bank'}, 1, model.nQuad));

% ===== Set initial quad euler =====
set(handles.uitable_quad_euler, 'Data', zeros(3, model.nQuad), ...
    'ColumnFormat', repmat({'bank'}, 1, model.nQuad));

% ===== Set initial quad input =====
set(handles.uitable_quad_input, 'Data', zeros(6, model.nQuad), ...
    'ColumnFormat', repmat({'bank'}, 1, model.nQuad));

% ===== Set initial quad goal =====
table_quad_goal_temp = cfg.quad.goal;
table_quad_goal_temp(4, :) = rad2deg(cfg.quad.goal(4, :));
set(handles.uitable_quad_goal, 'Data', table_quad_goal_temp, ...
    'ColumnFormat', repmat({'bank'}, 1, model.nQuad));
if cfg.setParaGui == 1
    set(handles.uitable_quad_goal, 'ColumnEditable', true(1, model.nQuad));
end

% ===== Set initial collision avoidance parameters =====
set(handles.uitable_mpc_coll, 'Data', [cfg.quad.coll, cfg.obs.coll]);
if cfg.setParaGui == 1
    set(handles.uitable_mpc_coll, 'ColumnEditable', true(1, 2));
end

% ===== Set initial weights
set(handles.uitable_mpc_weights, 'Data', [cfg.weightStage, cfg.weightN]);
if cfg.setParaGui == 1
    set(handles.uitable_mpc_weights, 'ColumnEditable', true(1, 2));
end

% ===== Check flag =====
set(handles.checkbox_show_quad_size, 'Value', cfg.ifShowQuadSize);
set(handles.checkbox_show_quad_path, 'Value', cfg.ifShowQuadPath);
set(handles.checkbox_show_quad_goal, 'Value', cfg.ifShowQuadGoal);
set(handles.checkbox_show_quad_head, 'Value', cfg.ifShowQuadHead);
set(handles.checkbox_show_quad_cov, 'Value', cfg.ifShowQuadCov);
set(handles.checkbox_show_quad_pathcov, 'Value', cfg.ifShowQuadPathCov);

% ===== Running para initilization =====
handles.UserData.quad_goal   = cfg.quad.goal;
handles.UserData.mpc_coll    = [cfg.quad.coll, cfg.obs.coll];
handles.UserData.mpc_weights = [cfg.weightStage, cfg.weightN];

handles.UserData.flagNewGuiPara = 0;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSwap  = 0;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes main_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


%% --- Outputs from this function are returned to the command line.
function varargout = main_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


%% --- Executes on button press in checkbox_show_quad_size.
function checkbox_show_quad_size_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_show_quad_size (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_show_quad_size
if get(hObject, 'Value')
    handles.UserData.cfg.ifShowQuadSize = 1;
else
    handles.UserData.cfg.ifShowQuadSize = 0;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in checkbox_show_quad_cov.
function checkbox_show_quad_cov_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_show_quad_cov (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_show_quad_cov
if get(hObject, 'Value')
    handles.UserData.cfg.ifShowQuadCov = 1;
else
    handles.UserData.cfg.ifShowQuadCov = 0;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in checkbox_show_quad_path.
function checkbox_show_quad_path_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_show_quad_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_show_quad_path
if get(hObject, 'Value')
    handles.UserData.cfg.ifShowQuadPath = 1;
else
    handles.UserData.cfg.ifShowQuadPath = 0;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in checkbox_show_quad_pathcov.
function checkbox_show_quad_pathcov_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_show_quad_pathcov (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_show_quad_pathcov
if get(hObject, 'Value')
    handles.UserData.cfg.ifShowQuadPathCov = 1;
else
    handles.UserData.cfg.ifShowQuadPathCov = 0;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in checkbox_show_quad_goal.
function checkbox_show_quad_goal_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_show_quad_goal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_show_quad_goal
if get(hObject, 'Value')
    handles.UserData.cfg.ifShowQuadGoal = 1;
else
    handles.UserData.cfg.ifShowQuadGoal = 0;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in checkbox_show_quad_head.
function checkbox_show_quad_head_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox_show_quad_head (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox_show_quad_head
if get(hObject, 'Value')
    handles.UserData.cfg.ifShowQuadHead = 1;
else
    handles.UserData.cfg.ifShowQuadHead = 0;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes when entered data in editable cell(s) in uitable_quad_goal.
function uitable_quad_goal_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable_quad_goal (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

% Verify that edited value is a valid number and is within workspace
handles.UserData.flagNewGuiPara = 0;
dataOK = true;
ws.xdim = [-handles.UserData.cfg.ws(1), handles.UserData.cfg.ws(1)];
ws.ydim = [-handles.UserData.cfg.ws(2), handles.UserData.cfg.ws(2)];
ws.zdim = [0.5, handles.UserData.cfg.ws(3)];
newVal = eventdata.NewData;
newIndices = eventdata.Indices;
% verify the number
if isnumeric(newVal) && ~isnan(newVal)
    switch newIndices(1)
        case 1
            if (newVal < ws.xdim(1) || newVal > ws.xdim(2))
                dataOK = false;
            end
        case 2
            if (newVal < ws.ydim(1) || newVal > ws.ydim(2))
                dataOK = false;
            end        
        case 3
            if (newVal < ws.zdim(1) || newVal > ws.zdim(2))
                dataOK = false;
            end
        case 4
            if (newVal < -180 || newVal > 180)
                dataOK = false;
            end
    end
else
    dataOK = false;
end
% if the data is not okay, display non-blocking message and reset value
if ~dataOK
     msgbox('Quad goal outside workspace. Value reset.', 'Error','error');
     hObject.Data(newIndices(1),newIndices(2)) = eventdata.PreviousData;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes when entered data in editable cell(s) in uitable_mpc_coll.
function uitable_mpc_coll_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable_mpc_coll (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

% Verify that edited value is a valid number
handles.UserData.flagNewGuiPara = 0;
dataOK = true;
newVal = eventdata.NewData;
newIndices = eventdata.Indices;
% verify the number
if isnumeric(newVal) && ~isnan(newVal)
    switch newIndices(1)
        case 1
            if (newVal <= 0)
                dataOK = false;
            end
        case 2
            if (newVal < 1)
                dataOK = false;
            end 
    end
else
    dataOK = false;
end
% if the data is not okay, display non-blocking message and reset value
if ~dataOK
     msgbox('Collision avoidance parameter not valid. Value reset.', 'Error','error');
     hObject.Data(newIndices(1),newIndices(2)) = eventdata.PreviousData;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes when entered data in editable cell(s) in uitable_mpc_weights.
function uitable_mpc_weights_CellEditCallback(hObject, eventdata, handles)
% hObject    handle to uitable_mpc_weights (see GCBO)
% eventdata  structure with the following fields (see MATLAB.UI.CONTROL.TABLE)
%	Indices: row and column indices of the cell(s) edited
%	PreviousData: previous data for the cell(s) edited
%	EditData: string(s) entered by the user
%	NewData: EditData or its converted form set on the Data property. Empty if Data was not changed
%	Error: error string when failed to convert EditData to appropriate value for Data
% handles    structure with handles and user data (see GUIDATA)

% Verify that edited value is a valid number
handles.UserData.flagNewGuiPara = 0;
dataOK = true;
newVal = eventdata.NewData;
newIndices = eventdata.Indices;
% verify the number
if isnumeric(newVal) && ~isnan(newVal)
    switch newIndices(1)
        case 1
            if (newVal < 0)
                dataOK = false;
            end
        case 2
            if (newVal < 0)
                dataOK = false;
            end 
        case 3
            if (newVal < 0)
                dataOK = false;
            end
        case 4
            if (newVal < 0)
                dataOK = false;
            end 
    end
else
    dataOK = false;
end
% if the data is not okay, display non-blocking message and reset value
if ~dataOK
     msgbox('Cost weights should be larger than zero. Value reset.', 'Error','error');
     hObject.Data(newIndices(1),newIndices(2)) = eventdata.PreviousData;
end
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in pushbutton_set.
function pushbutton_set_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Set all running para
handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSWap  = 1;
handles.UserData.quad_goal   = get(handles.uitable_quad_goal, 'Data');
handles.UserData.quad_goal(4, :) = deg2rad(handles.UserData.quad_goal(4, :));
handles.UserData.mpc_coll    = get(handles.uitable_mpc_coll, 'Data');
handles.UserData.mpc_weights = get(handles.uitable_mpc_weights, 'Data'); 
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in pushbutton_reset.
function pushbutton_reset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_reset (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Reset all running para
handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSWap  = 0;
handles.UserData.quad_goal   = handles.UserData.cfg.quad.goal;
handles.UserData.mpc_coll    = [handles.UserData.cfg.quad.coll, ...
                                handles.UserData.cfg.obs.coll];
handles.UserData.mpc_weights = [handles.UserData.cfg.weightStage, ...
                                handles.UserData.cfg.weightN]; 
% save the data to the (global) handles structure
guidata(hObject, handles);


%% --- Executes on button press in pushbutton_swap.
function pushbutton_swap_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_swap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSWap  = 0;
handles.UserData.quad_goal   = handles.UserData.quad_goal;
handles.UserData.quad_goal(1:2, :) = -handles.UserData.quad_goal(1:2, :);
% save the data to the (global) handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_random.
function pushbutton_random_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_random (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global model
handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSWap  = 0;
xDim = [-handles.UserData.cfg.ws(1) + 0.5, handles.UserData.cfg.ws(1) - 0.5];
yDim = [-handles.UserData.cfg.ws(2) + 0.5, handles.UserData.cfg.ws(2) - 0.5];
zDim = [0.6, 2.1];
[~, ~, quadEndPos] = scn_random(model.nQuad, xDim, yDim, zDim);
handles.UserData.quad_goal = quadEndPos;
% save the data to the (global) handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_keep_swap.
function pushbutton_keep_swap_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_keep_swap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 1;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSWap  = 0;
% save the data to the (global) handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_keep_random.
function pushbutton_keep_random_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_keep_random (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 1;
handles.UserData.flagEightSWap  = 0;
% save the data to the (global) handles structure
guidata(hObject, handles);


% --- Executes on button press in pushbutton_eight_swap.
function pushbutton_eight_swap_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_eight_swap (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.UserData.flagNewGuiPara = 1;
handles.UserData.flagKeepSwap   = 0;
handles.UserData.flagKeepRandom = 0;
handles.UserData.flagEightSwap  = 1;
% save the data to the (global) handles structure
guidata(hObject, handles);
