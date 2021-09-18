function varargout = milp_path_gui(varargin)
% MILP_PATH_GUI Graphical front end for 2-D Path-Planning
%      
% For more information, run MILP_PATH_GUI and use the "Help" menu.
%
% Version 1.0 : A.G.Richards, 16 July 2007
%
% See also: MILP_PATH
%

% Edit the above text to modify the response to help milp_path_gui

% Last Modified by GUIDE v2.5 13-Jul-2007 16:42:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @milp_path_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @milp_path_gui_OutputFcn, ...
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


% --- Executes just before milp_path_gui is made visible.
function milp_path_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to milp_path_gui (see VARARGIN)

% Choose default command line output for milp_path_gui
handles.output = hObject;

% initialise problem storage variables
initialise_problem(hObject,handles)

% --- Outputs from this function are returned to the command line.
function varargout = milp_path_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --------------------------------------------------------------------
function File_New_Callback(hObject, eventdata, handles)
% hObject    handle to File_New (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if wipewarning==1,
    
    % initialise problem storage variables
    initialise_problem(hObject,handles)

end

% --------------------------------------------------------------------
function File_Open_Callback(hObject, eventdata, handles)
% hObject    handle to File_Open (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if wipewarning==1,
    
    % initialise problem storage variables
    handles = load_problem(hObject,handles);

end

% --------------------------------------------------------------------
function File_Save_Callback(hObject, eventdata, handles)
% hObject    handle to File_Save (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

save_problem(handles)

% --------------------------------------------------------------------
function File_Exit_Callback(hObject, eventdata, handles)
% hObject    handle to File_Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

exit_gui(handles)

% --------------------------------------------------------------------
function Obstacle_AddSq_Callback(hObject, eventdata, handles)
% hObject    handle to Obstacle_AddSq (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

add_square(hObject,handles)

% --------------------------------------------------------------------
function Obstacle_AddRect_Callback(hObject, eventdata, handles)
% hObject    handle to Obstacle_AddRect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

add_rect(hObject,handles)

% --------------------------------------------------------------------
function Obstacle_DelObs_Callback(hObject, eventdata, handles)
% hObject    handle to Obstacle_DelObs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

delete_obstacle(hObject,handles)

% --------------------------------------------------------------------
function File_Menu_Callback(hObject, eventdata, handles)
% hObject    handle to File_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Obstacle_Menu_Callback(hObject, eventdata, handles)
% hObject    handle to Obstacle_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function Obstacle_SetLimits_Callback(hObject, eventdata, handles)
% hObject    handle to Obstacle_SetLimits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_limits(hObject,handles)

% --------------------------------------------------------------------
function Vehicle_Menu_Callback(hObject, eventdata, handles)
% hObject    handle to Vehicle_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Veh_Setinitpos_Callback(hObject, eventdata, handles)
% hObject    handle to Veh_Setinitpos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_initpos(hObject,handles)

% --------------------------------------------------------------------
function Veh_setinitvel_Callback(hObject, eventdata, handles)
% hObject    handle to Veh_setinitvel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_initvel(hObject,handles)

% --------------------------------------------------------------------
function Veh_settargbox_Callback(hObject, eventdata, handles)
% hObject    handle to Veh_settargbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_targbox(hObject,handles)

% --------------------------------------------------------------------
function Solve_Menu_Callback(hObject, eventdata, handles)
% hObject    handle to Solve_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% --------------------------------------------------------------------
function Solve_solve_Callback(hObject, eventdata, handles)
% hObject    handle to Solve_solve (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

solve_problem(handles)

% --------------------------------------------------------------------
function Solve_setsteps_Callback(hObject, eventdata, handles)
% hObject    handle to Solve_setsteps (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_numsteps(hObject,handles)

% --------------------------------------------------------------------
function Veh_SetVehLimits_Callback(hObject, eventdata, handles)
% hObject    handle to Veh_SetVehLimits (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

set_vehlimits(hObject,handles)

% --------------------------------------------------------------------
function Help_Menu_Callback(hObject, eventdata, handles)
% hObject    handle to Help_Menu (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Help_Gui_Callback(hObject, eventdata, handles)
% hObject    handle to Help_Gui (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

web(['file:///' pwd '/milp_help.html'],'-browser')

% --------------------------------------------------------------------
function Help_Solver_Callback(hObject, eventdata, handles)
% hObject    handle to Help_Solver (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

help milp_path

% --------------------------------------------------------------------
function Help_About_Callback(hObject, eventdata, handles)
% hObject    handle to Help_About (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

msgbox('MILP_PATH_GUI Version 1.0')

% --------------------------------------------------------------------
function Help_Agr_Callback(hObject, eventdata, handles)
% hObject    handle to Help_Agr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

web('http://seis.bris.ac.uk/~aeagr/software.html','-browser')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Functional stuff down here
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% show "msg" in the status message box
function msgtext(handles,msg)
set(handles.text1,'String',msg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function yes = wipewarning

% warning dialog
buttonname=questdlg('Action will wipe current problem. Are you sure?',...
    'Warning','Yes','No','Yes');

% if said yes,
if strcmp(buttonname,'Yes'),
    yes = 1;
else,
    yes = 0;
end
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xs] = vertices(A,B,M)
%
% Xs = vertices(A,B,M)
%
% find vertices of Ax<=B, |x|<M, (2-D only)
%
% 2-D only!!!!!!

% set up initial vertex list
Xs_tmp = M*[-1 -1 1 1;
        -1 1 -1 1]';

% work through each constraint
for ii=[1:(size(A,1)-1)],
  
  % work through all lower constraints
  for jj=[(1+ii):size(A,1)],
  
    % find intersection
    Aint = [A(ii,:); A(jj,:)];
    if rank(Aint)>1,

      % lines do intersect
      x = Aint \ [B(ii,:); B(jj,:)];
    
      % add vertex to new vertex list
      Xs_tmp = [Xs_tmp; x'];
    else
        disp('rank is smaller than or equal to 1')
    end
    
  end % continue finding vertices
  
end

% now remove all those outside
ind = find(max(A*Xs_tmp'-B*ones(1,size(Xs_tmp,1)))<=1e-6);
Xs = Xs_tmp(ind,:);

% bail if no points
if min(size(Xs))<2,
  return
end

% re-order for plotting
% first find centroid
xc = mean(Xs);

% angles around centroid
angs = atan2(Xs(:,1) - xc(1),Xs(:,2) - xc(2));

% sort angles
[asort,isort] = sort(angs);

% sort vertices
Xs = Xs(isort,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function exit_gui(handles)

% close the GUI
close(handles.figure1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function add_rect(hObject,handles)

% set text to first click message
msgtext(handles,'Click on one corner of new obstacle')

% get centre click
[xc,yc] = ginput(1);

% temporary plot of it
hc = plot(xc,yc,'gx');

% set text to second click message
msgtext(handles,'Click on opposite corner')

% get centre click
[xb,yb] = ginput(1);

% clear the contre mark
delete(hc)

% convert to min-max form
xmin = min([xb xc]);
xmax = max([xb xc]);
ymin = min([yb yc]);
ymax = max([yb yc]);

% get current number of obstacles
No = length(handles.probdata.obsets);

% convert to set
handles.probdata.obsets{No+1} = [-1  0 -xmin;
                                  1  0 xmax;
                                  0 -1 -ymin;
                                  0  1 ymax];

% store new data                      
guidata(hObject, handles);

% set text to final message
msgtext(handles,'Obstacle added')

% redraw
refresh_drawing(handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function add_square(hObject,handles)

% set text to first click message
msgtext(handles,'Click at the centre of the square')

% get centre click
[xc,yc] = ginput(1);

% temporary plot of it
hc = plot(xc,yc,'gx');

% set text to second click message
msgtext(handles,'Click somewhere on the edge of the square')

% get centre click
[xb,yb] = ginput(1);

% get radius
r = max(abs([xb-xc; yb-yc]));

% clear the contre mark
delete(hc)

% convert to min-max form
xmin = xc-r;
xmax = xc+r;
ymin = yc-r;
ymax = yc+r;

% get current number of obstacles
No = length(handles.probdata.obsets);

% convert to set
handles.probdata.obsets{No+1} = [-1  0 -xmin;
                                  1  0 xmax;
                                  0 -1 -ymin;
                                  0  1 ymax];

% store new data                      
guidata(hObject, handles);

% set text to final message
msgtext(handles,'Obstacle added')

% redraw
refresh_drawing(handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function set_limits(hObject,handles)

% default answers
xmind = num2str(handles.probdata.limits(1));
xmaxd = num2str(handles.probdata.limits(2));
ymind = num2str(handles.probdata.limits(3));
ymaxd = num2str(handles.probdata.limits(4));

% get input
answer = inputdlg({'Xmin','Xmax','Ymin','Ymax'},'Set Area Limits',1,{xmind,xmaxd,ymind,ymaxd});

% bail out if user cancelled
if length(answer)~=4,
  msgtext(handles,'Cancelled')
  return
end

% convert to vector
xmin = str2num(answer{1});
xmax = str2num(answer{2});
ymin = str2num(answer{3});
ymax = str2num(answer{4});
inputValue = [xmin xmax ymin ymax]';

% check length
if length(inputValue)==4 && inputValue(2)>inputValue(1) && inputValue(4)>inputValue(3),
    axis(handles.axes1,inputValue)
    handles.probdata.limits = inputValue;
    guidata(hObject,handles)
else
    msgtext(handles,'Invalid axis vector')
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function set_vehlimits(hObject,handles)

% default answers
vmaxd = num2str(handles.probdata.vmax);
vmind = num2str(handles.probdata.vmin);
amaxd = num2str(handles.probdata.amax);

% get input
answer = inputdlg({'Vmax','Vmin','Amax'},'Set Vehicle Limits',1,{vmaxd,vmind,amaxd});

% bail out if user cancelled
if length(answer)~=3,
  msgtext(handles,'Cancelled')
  return
end

% convert to vector
vmaxn = str2num(answer{1});
vminn = str2num(answer{2});
amaxn = str2num(answer{3});

% check and store
if prod(size(vmaxn))==1 & vmaxn>0,
    handles.probdata.vmax = vmaxn;
end
if prod(size(vminn))==1 & vminn>=0 & vminn<vmaxn,
    handles.probdata.vmin = vminn;
end
if prod(size(amaxn))==1 & amaxn>0,
    handles.probdata.amax = amaxn;
end
guidata(hObject,handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function delete_obstacle(hObject, handles)

% set text to first click message
msgtext(handles,'Click in the obstacle to delete')

% get centre click
[xd,yd] = ginput(1);

% default message
msg = 'No matching obstacle found';

% loop through obstacles
for ii=[1:length(handles.probdata.obsets);],
    
    % this obstacle
    A = handles.probdata.obsets{ii}(:,1:end-1);
    B = handles.probdata.obsets{ii}(:,end);
        
    % check if clicked inside
    if A*[xd;yd]<=B,
        % wipe it
        handles.probdata.obsets(ii) = [];
        % update storage
        guidata(hObject, handles);
        % set msg
        msg = 'Obstacle deleted';
        % redraw
        refresh_drawing(handles);
        % done looping
        break
    end
end

% show message text
msgtext(handles,msg)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function set_initpos(hObject,handles)

% set text to first click message
msgtext(handles,'Click at initial vehicle location')

% get centre click
[xd,yd] = ginput(1);

% store it
handles.probdata.initpos = [xd yd]';
guidata(hObject,handles)

refresh_drawing(handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function set_initvel(hObject,handles)

% set text to first click message
msgtext(handles,'Click at the point the vehicle would reach in one time step')

% get centre click
[xd,yd] = ginput(1);

vel = ([xd yd]' - handles.probdata.initpos)/handles.probdata.dt;

% store it
handles.probdata.initvel = vel;
guidata(hObject,handles)

refresh_drawing(handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function set_targbox(hObject,handles)

% set text to first click message
msgtext(handles,'Click on corner of target box')

% get centre click
[xc,yc] = ginput(1);

% temporary plot of it
hc = plot(xc,yc,'gx');

% set text to second click message
msgtext(handles,'Click on opposite corner')

% get centre click
[xb,yb] = ginput(1);

% clear the centre mark
delete(hc)

% convert to min-max form
xmin = min([xb xc]);
xmax = max([xb xc]);
ymin = min([yb yc]);
ymax = max([yb yc]);

% store it
handles.probdata.targbox = [xmin xmax ymin ymax]';
guidata(hObject,handles)

refresh_drawing(handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function save_problem(handles)

% copy obstacle data into nicer variable
probdata = handles.probdata;
uisave({'probdata'},'obstacle.mat');

msgtext(handles,'Saved file')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function handles = load_problem(hObject,handles)

% load a file
uiload

% check for necessary variable
if ~exist('probdata'),
  msgtext(handles,'Invalid problem file')
  return
end
    
% check its a structure
if ~isstruct(probdata),
  msgtext(handles,'Invalid problem data variable')
  return
end
    
% check its got all the fields we need
fldchk = isfield(probdata,fieldnames(handles.probdata));
if sum(fldchk)<length(fldchk),
  msgtext(handles,'Invalid problem data fields')
  return
end
    
% copy into structure and store
handles.probdata = probdata;
guidata(hObject, handles)
    
% draw initial problem
refresh_drawing(handles)

% message
msgtext(handles,'Problem loaded')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function initialise_problem(hObject, handles)

% new store for my data
handles.probdata.obsets = {[-1 0 1; 1 0 1; 0 -1 1; 0 1 1]}; % cell array of obstacle polygons
handles.probdata.limits = [-5 5 -5 5]'; % max operating area
handles.probdata.initpos=[-2 0]'; % initial position
handles.probdata.initvel=[0.5 0.5]'; % initial velocity
handles.probdata.targbox=[2.5 3.5 -0.5 0.5]'; % target region [xmin xmax ymin ymax]
handles.probdata.vmax=1; % max speed
handles.probdata.vmin=0.8; % min speed
handles.probdata.amax=1; % max acceleration
handles.probdata.dt = 1; % time step length
handles.probdata.Nt = 10; % number of time steps

% Update handles structure
guidata(hObject, handles);

% draw initial problem
refresh_drawing(handles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function refresh_drawing(handles)

% clear axes first
cla(handles.axes1)

% big M for vertex hunting
M = 1000;

% loop through obstacles
for ii=1:length(handles.probdata.obsets),
   
    % this obstacle
    A = handles.probdata.obsets{ii}(:,1:end-1);
    B = handles.probdata.obsets{ii}(:,end);
    
    % get vertices
    Xs = vertices(A,B,M);
    
    % plot the thing
    patch(Xs(:,1),Xs(:,2),'r');
    
end

% set limits
axis(handles.probdata.limits)

% initial position
plot(handles.probdata.initpos(1),handles.probdata.initpos(2),'bx')

% initial velocity
plot(handles.probdata.initpos(1)+[0 handles.probdata.dt*handles.probdata.initvel(1)], ...
     handles.probdata.initpos(2)+[0 handles.probdata.dt*handles.probdata.initvel(2)],'b-')

% target box
patch(handles.probdata.targbox([1 2 2 1]),handles.probdata.targbox([4 4 3 3]),'g')
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function set_numsteps(hObject,handles)

answer = inputdlg({'Number of time steps' 'Time step length'},'Time Steps',1,{num2str(handles.probdata.Nt) num2str(handles.probdata.dt)});

% convert
valn = str2num(answer{1});
valt = str2num(answer{2});

% check it
if length(valn)==1 && length(valt)==1,
    if valn==floor(valn),
        % store
        handles.probdata.Nt = valn;
        handles.probdata.dt = valt;
        guidata(hObject, handles)
        % message
        msgtext(handles,['Now set to ' num2str(valn) ' time steps of length ' num2str(valt) ' units.'])
        % redraw, as velocity vec changes with dt
        refresh_drawing(handles)
    else,
        msgtext(handles,['Invalid number of time steps'])
    end
else,
    msgtext(handles,['Invalid time step data'])
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function solve_problem(handles)

% starting message
msgtext(handles,['Starting solver'])

% store the time
start_time = clock;

% run the solver
[rs,vs,as,flag] = milp_path(handles.probdata.initpos, ...
                            handles.probdata.initvel, ...
                            handles.probdata.targbox, ...
                            handles.probdata.limits, ...
                            handles.probdata.obsets, ...
                            handles.probdata.vmax, ...
                            handles.probdata.vmin, ...
                            handles.probdata.amax, ...
                            handles.probdata.dt, ...
                            handles.probdata.Nt);

% did it work?                        
if flag==1,
    
    % plot the thing
    plot(rs(1,:),rs(2,:),'c.-')
    
    % show a message
    msgtext(handles,['Problem solved in ' num2str(etime(clock,start_time)) 's'])
    
else,
    
    % show a message
    msgtext(handles,['Problem could not be solved'])
    
end
    
    














