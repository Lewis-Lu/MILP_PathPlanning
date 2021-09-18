function [flag,varargout] = glpkread(fname,varargin)
%
% [flag,var1,var2,...,varN]=glpkread(fname,varname1,varname2,...,varnameN)
%
% GLPKREAD: read data from GLPK output file
%
% fname is the filename
%
% varname's are strings with the names of variables to look for
%
% 'flag' returns 1 if "Status: OPTIMAL" line found, 0 otherwise
%
% Outputs those variables. Empty if the variable not found in the
% file. Example:
%
% [flag,x,y] = glpkread('data.txt','x','y')
%
% returns x and y if they're present in the file 'data.txt'.
%
% Note - requires Matlab 7 or higher for regexp handling
%
% 24 Aug 2005 : Arthur Richards : New utility
% 22 Sep 2005 : Arthur Richards : Added flag functionality
% 16 Oct 2006 : Arthur Richards : Extended for up to 4-D arrays
% 07 Jun 2007 : Arthur Richards : Fixed so it can handle scalars
% 29 Feb 2008 : Arthur Richards : Fixed flag function for "INTEGER OPTIMAL"

% die if no var list
if nargin<2,
  error('glpkread must have some variables to look for')
end

% die if wrong number of outputs
if nargout~=nargin,
  error('glpkread should have output for each requested variable')
end

% open file
fid = fopen(fname,'r');

% die on file error
if fid==-1,
  error('glpkread cannot open file')
end

% set up empty output
varargout = cell(nargout,1);

% initialize optimality flag
flag = 0;

% set up the matching pattern
patt1 = '(?<lineno>\w+)\s+(?<varname>';
patt2 = varargin{1};
for vv=[3:nargin],
  patt2 = [patt2 '|' varargin{vv-1}];
end
patt3 = ')\[(?<indices>.+)\]([^\-0-9]+)(?<value>[\-\.0-9e]+)';
pattern = strcat(patt1,patt2,patt3);

% alternative pattern in case its a scalar
patt3b = ')\s+([^\-0-9]+)(?<value>[\-\.0-9e]+)';
patternb = strcat(patt1,patt2,patt3b);

% loop through file
while 1

  % read line
  s = fgetl(fid);
  
  % append next line to fix multi-line entries
  thislen = length(s);
  if thislen>0,
    if s(length(s))==']',
        s2 = fgetl(fid);
        s = [s s2];
    end
  end
  
  
  % breakout if end of file
  if ~ischar(s), 
    break, 
  end
 
  % see if its the optimality flag
  a = strfind(s,'Status:     OPTIMAL');
  if ~isempty(a),
    flag = 1;
  end
  a = strfind(s,'Status:     INTEGER OPTIMAL');
  if ~isempty(a),
    flag = 1;
  end
  
  % scan the input line
  a = regexp(s,pattern,'names');
  
  % AGR debugging
  %disp(['Line: ' s])
  %disp(a)
  
  % process if line works
  if ~isempty(a),
    
    % check if its a var we're looking for
    nout = strmatch(a.varname,varargin,'exact');
    if ~isempty(nout),
      
      % scan for indices, padding with ones if needed
      ijk = sscanf([a.indices ',1,1,1,1'],'%d,%d,%d,%d');
      
      % debug
      %disp(['Line: ' s])
      %disp(ijk)
      
      % enter into output matrices
      varargout{nout}(ijk(1),ijk(2),ijk(3),ijk(4)) = ...
	    str2double(a.value);
      
    end
    
  else,

    % check against alternative pattern in case its a scalar

    % scan the input line
    a = regexp(s,patternb,'names');
  
    % process if line works
    if ~isempty(a),
    
      % check if its a var we're looking for
      nout = strmatch(a.varname,varargin,'exact');
      if ~isempty(nout),
	
	% debug
	%disp(['Line: ' s])
	
	% enter into output matrices
	varargout{nout} = str2double(a.value);
      
      end
      
    end
    
  end
  
end

% close file
fclose(fid);