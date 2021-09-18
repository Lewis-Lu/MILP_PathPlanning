function [flag, varargout] = analysisSolution(fname, varargin)

% [flag, varargout] = analysisSolution(fname, varargin)
% 
% extract the interesting field from the solution text file that is
% outputed by the solver

if nargin < 2
    error('Input the query field name')
end

if nargin ~= nargout
    error('Dimension of the output and input should be the same')
end

fid = fopen(fname, 'r');

if fid == -1
    error('Analysis module cannot open the file')
end

varargout = cell(nargin, 1);

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

end


