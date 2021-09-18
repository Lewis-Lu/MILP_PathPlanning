function count=AMPLmatrix(fid,pname,p,varargin)
%
%   function count=AMPLmatrix(fid,pname,p,[rowheads,colheads])
%
% Write matrix of floats to AMPL data file
%
%   fid   : file handle of the data file from 'fopen' 
%   pname : name to be given to the parameter in the file (string)
%   p     : the value of the parameter (matrix)
%
%   rowheads (optional) : row indices
%   colheads (optional) : column indices
%
%   If the matrix is M x N, rowheads should be 1 x N and colheads
%   should be 1 x M. If omitted, rows are labeled 1 to N and
%   columns 1 to M. Ths facility allows zero-based indices to be
%   used in the AMPL parameter, or any other index system
%
%   count : number of bytes written
%
% Copyright A. Richards, MIT, 2002
%

s=size(p);c=0;

if size(s,2)>2,
  error('Not matrix')
else   
  % matrix
  w = size(p,2);
  % get column headers
  colhdr=[1:w];
  if nargin>3,
    colhdr=varargin{2};
  end
  % print parameter name
  c = c + fprintf(fid,['param ' pname ' : ']);
  % print column headers
  for j=[1:w],
    c = c + fprintf(fid,'%20.0f',colhdr(j));
  end
  % and the equals
  c = c + fprintf(fid,'\t:=\n');
  % get row headers
  h = size(p,1);
  rowhdr=[1:h];
  if nargin>3,
    rowhdr=varargin{1};
  end
  % print rows
  for i=[1:(h-1)],
    % print row header
    c = c + fprintf(fid,'\t%4.0f',rowhdr(i));
    % print row data
    for j=[1:w],
      c = c + fprintf(fid,'%20.14f',p(i,j));
    end
    % new line
    c = c + fprintf(fid,'\n');
  end
  % do last row separate
  c = c + fprintf(fid,'\t%4.0f',rowhdr(h));
  for j=[1:w],
    c = c + fprintf(fid,'%20.14f',p(h,j));
  end
  % end with semicolon
  c = c + fprintf(fid,'\t;\n');
end
count=c;