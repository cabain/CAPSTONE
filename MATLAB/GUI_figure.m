function varargout = GUI_figure(varargin)
%Iinitialization code - DO NOT EDIT....ha I edited
% --- Executes just before GUI_figure is made visible.
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @GUI_figure_OpeningFcn, ...
    'gui_OutputFcn',  @GUI_figure_OutputFcn, ...
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
function GUI_figure_OpeningFcn(hObject, eventdata, handles, varargin)
% Choose default command line output for GUI_figure
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);



% --- Outputs from this function are returned to the command line.
function varargout = GUI_figure_OutputFcn(hObject, eventdata, handles)
% Get default command line output from handles structure
varargout{1} = handles.output;
%global variables
global yawHeading;
%define yawHeading and assign it to the workspace
assignin('base', 'yawHeading', yawHeading);
%Initialize Comms Ports
delete(instrfind);
global serialObject;
serialObject = serial('COM6');
fopen(serialObject);
%Declare a timer
global tmr;
tmr = timer('Name','Reminder',...
    'Period',1,...  % Update the time every deci-second.
    'TasksToExecute',inf,...  % number of times to update
    'ExecutionMode','fixedDelay',...
    'TimerFcn',{@updater});
start(tmr);  % Start the timer object.
set(hObject,'deletefcn',{@deleter})  % Kill timer if fig is closed.



%Runs when the timer ticks
function [] = updater(varargin)
%Stream of Data from CubeSAT
global yawHeading;
global serialObject;
dataStream = fgetl(serialObject);
%assign the data to an array if it is an integer
newYawHeading = str2double(dataStream);
if(isnan(newYawHeading))
    disp(dataStream); %display the strings
else
    yawHeading = [yawHeading, newYawHeading]; %add the heading to MATLAB
    %define yawHeading and assign it to the workspace
    assignin('base', 'yawHeading', yawHeading);
end


% --- Executes on button press in btnFrontForward.
function btnFrontForward_Callback(hObject, eventdata, handles)
global serialObject;
fprintf(serialObject, '%c', 'A');
set(handles.btnFrontForward, 'FontWeight', 'Bold');
set(handles.btnFrontReverse, 'FontWeight', 'normal');
set(handles.btnFrontToggleOff, 'FontWeight', 'normal');

% --- Executes on button press in btnFrontReverse.
function btnFrontReverse_Callback(hObject, eventdata, handles)
global serialObject;
fprintf(serialObject, '%c', 'B');
set(handles.btnFrontForward, 'FontWeight', 'normal');
set(handles.btnFrontReverse, 'FontWeight', 'Bold');
set(handles.btnFrontToggleOff, 'FontWeight', 'normal');

% --- Executes on button press in btnFrontToggleOff.
function btnFrontToggleOff_Callback(hObject, eventdata, handles)
global serialObject;
fprintf(serialObject, '%c', 'C');
set(handles.btnFrontToggleOff, 'FontWeight', 'Bold');
set(handles.btnFrontForward, 'FontWeight', 'normal');
set(handles.btnFrontReverse, 'FontWeight', 'normal');


% --- Executes on button press in btnSideForward.
function btnSideForward_Callback(hObject, eventdata, handles)
global serialObject;
fprintf(serialObject, '%c', 'D');
set(handles.btnSideForward, 'FontWeight', 'Bold');
set(handles.btnSideReverse, 'FontWeight', 'normal');
set(handles.btnSideToggleOff, 'FontWeight', 'normal');

% --- Executes on button press in btnSideReverse.
function btnSideReverse_Callback(hObject, eventdata, handles)
global serialObject;
fprintf(serialObject, '%c', 'E');
set(handles.btnSideForward, 'FontWeight', 'normal');
set(handles.btnSideReverse, 'FontWeight', 'Bold');
set(handles.btnSideToggleOff, 'FontWeight', 'normal');

% --- Executes on button press in btnSideToggleOff.
function btnSideToggleOff_Callback(hObject, eventdata, handles)
global serialObject;
fprintf(serialObject, '%c', 'F');
set(handles.btnSideToggleOff, 'FontWeight', 'Bold');
set(handles.btnSideForward, 'FontWeight', 'normal');
set(handles.btnSideReverse, 'FontWeight', 'normal');

function [] = deleter(varargin)
% If figure is deleted, so is timer.
global tmr;
stop(tmr);
delete(tmr);
%stop and close the serial port
global serialObject;
fclose(serialObject);
delete(instrfind);
%clear variables
clear global;
clear;
%display a closing message
disp('Successfully Closed.');
