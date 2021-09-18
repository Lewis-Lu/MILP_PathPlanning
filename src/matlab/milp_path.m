function [rs, vs, as, flag] = milp_path(initpos, ...
                                        initvel, ...
                                        targbox, ...
                                        opbox, ...
                                        obst, ...
                                        vmax, ...
                                        vmin, ...
                                        amax, ...
                                        dt, ...
                                        Nt)				    
%			    
% MILP_PATH Mixed-Integer Linear Programming 2-D Path Planner
%
% [rs, vs, as, flag] = MILP_PATH(initpos, ...
%                                initvel, ...
%				 targbox, ...
%				 opbox, ...
%				 obst, ...
%				 vmax, ...
%				 vmin, ...
%				 amax, ...
%				 dt, ...
%				 Nt)
%
% Searches for shortest path starting at position INITPOS and with
% velocity INITVEL and ending in the box specified by vector
% TARGBOX (as [Xmin Xmax Ymin Ymax]). The path must remain in the
% box OPBOX (same specification as TARGBOX).
%
% Obstacles are specified in OBST, a cell array of N-by-3 matrices
% representing polygons: OBST{ii}{:,1:2}*r <= OBST{ii}(:,3).
%
% VMAX, VMIN and AMAX are scalars representing maximum speed,
% minimum speed and maximum acceleration, respectively.
%
% The path is broken into NT steps each of length DT time units.
%
% FLAG returns 1 if the problem was solved successfully or 0
% otherwise. RS, VS and AS return the position, velocity and
% acceleration values, respectively.
%
% The following other utilities are required:
%  - the AMPL interface for Matlab
%  - the GLPSOL interface for Matlab
%  - GLPK, the Gnu Linear Programming Kit
% See http://seis.bris.ac.uk/~aeagr/software.html for details
%
% Version 1.0 : A.G.Richards, 16 July 2007
%
% Provided for research use only. Users are requested to cite the
% following paper in any work utilising this software: A. G. Richards
% and J. P. How, "Aircraft Trajectory Planning with Collision
% Avoidance using Mixed Integer Linear Programming," in Proceedings of
% the American Control Conference, 2002.
%
% See also MILP_PATH_GUI
%

% Modified by Lu, Hong in 2021.7.15

% ******************* Validate data *************************

model_root = 'D:\Postgraduate\Dissertation\src\mod\';
solver_root = 'D:\Postgraduate\Dissertation\src\glpk-5.0\w64\';
data_root = 'D:\Postgraduate\Dissertation\src\data\';
output_root = 'D:\Postgraduate\Dissertation\src\output\';

if size(initpos)~=[2 1],
  error('MILP_PATH : initial position argument must be 2 x 1')
end

if size(initvel)~=[2 1],
  error('MILP_PATH : initial velocity argument must be 2 x 1')
end

if size(targbox)~=[4 1],
  error('MILP_PATH : target box argument must be 4 x 1')
end

if (targbox(2)<=targbox(1)) || (targbox(4)<=targbox(3)),
  error('MILP_PATH : target box argument must be in form [Xmin Xmax Ymin Ymax]')
end

if size(opbox)~=[4 1],
  error('MILP_PATH : limit box argument must be 4 x 1')
end

if (opbox(2)<=opbox(1)) || (opbox(4)<=opbox(3)),
  error('MILP_PATH : limit box argument must be in form [Xmin Xmax Ymin Ymax]')
end

if length(obst)<1,
  error('MILP_PATH : must have at least one obstacle')
end

for ii=1:length(obst),
    if size(obst{ii},2)~=3,
        error('MILP_PATH : obst argument must be array of N x 3 matrices')
    end
end

if size(vmax)~=[1 1],
  error('MILP_PATH : maximum velocity must be scalar')
end

if size(vmin)~=[1 1],
  error('MILP_PATH : minimum velocity must be scalar')
end

if size(amax)~=[1 1],
  error('MILP_PATH : maximum acceleration must be scalar')
end

if size(dt)~=[1 1],
  error('MILP_PATH : time step length must be scalar')
end

if size(Nt)~=[1 1],
  error('MILP_PATH : number of time steps must be scalar')
end

if Nt~=floor(Nt),
  error('MILP_PATH : number of time steps must be integer')
end

% ******************* AMPL DATA FILE WRITE ******************

c = 0;
fid=fopen('milp_path.dat','w');
c = c + AMPLcomment(fid,'Matlab generated AMPL/GLPK data file\n');
c = c + AMPLcomment(fid,'');
c = c + AMPLcomment(fid,'For use with model milp_path.mod');
c = c + AMPLcomment(fid,'');

% circle approximations
Nv = 8; %for speed limits - reduce as binaries for min spd
c = c + AMPLscalarint(fid,'Nv',Nv);
Na = 20; %for acc limits
c = c + AMPLscalarint(fid,'Na',Na);
% sine and cosine tables
c = c + AMPLvector(fid,'costa',cos(2*pi*[1:Na]/Na));
c = c + AMPLvector(fid,'sinta',sin(2*pi*[1:Na]/Na));
c = c + AMPLvector(fid,'costv',cos(2*pi*[1:Nv]/Nv));
c = c + AMPLvector(fid,'sintv',sin(2*pi*[1:Nv]/Nv));

% speed and acc limits
c = c + AMPLscalar(fid,'amax',amax);
c = c + AMPLscalar(fid,'vmax',vmax);
c = c + AMPLscalar(fid,'vmin',vmin);

% number of timesteps
c = c + AMPLscalarint(fid,'Nt',Nt);

% time step length
c = c + AMPLscalar(fid,'dt',dt);

% initial conditions
c = c + AMPLvector(fid,'ri',initpos);
c = c + AMPLvector(fid,'vi',initvel);

% target
c = c + AMPLvector(fid,'rf',targbox);

% operating box
c = c + AMPLvector(fid,'ro',opbox);

% control weight - low means min time
c = c + AMPLscalar(fid,'gamma',0.001);

% compile the obstacle data into a constraint list 
% [obst_num p_x_i p_y_i q_i]
obcons = [];
for ii=1:length(obst),
  obcons = [obcons; [ii*ones(size(obst{ii},1),1) obst{ii}]];
end
c = c + AMPLscalar(fid,'No',length(obst));
c = c + AMPLscalar(fid,'Nc',size(obcons,1));
c = c + AMPLmatrix(fid,'obst',obcons);

% compile "big M's" for tighter relaxations
M = max((obcons(:,4)*ones(1,4) - obcons(:,2:3)*opbox([1 1 2 2; 3 4 3 4]))')';
c = c + AMPLvector(fid,'M',M);

% completed writing file
fclose(fid);
sprintf('%d bytes written',c)

% path configuration
solver = [solver_root 'glpsol.exe '];

model = [model_root 'milp_path.mod '];

data = [data_root 'milp_path.dat '];

ofile = [output_root 'milp_path.txt '];

% formulate the command for the os
cmd = [solver '--model ' model '--data ' data '--output ' ofile];

statue = system(cmd);

if statue == 0
    disp('solver is called.')
else
    disp('solver is not called.')
end


% read GLPK variables "r" and "v" into Matlab "rs" and "vs"
[flag,rs,vs,as,fs]=glpkread('milp_path.txt','r','v','a','f');

% bail here if it didn't solve
if flag==0,
  return
end

% get actual length
len = find(fs);

% truncate to correct length
rs = rs(:,1:len);
vs = vs(:,1:len);
as = as(:,1:(len-1));