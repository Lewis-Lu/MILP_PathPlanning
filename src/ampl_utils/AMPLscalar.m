function count=AMPLscalar(fid,pname,p)
%
%   function count=AMPLscalar(fid,pname,p)
%
% Write floating point scalar to AMPL data file
%
%   fid   : file handle of the data file from 'fopen' 
%   pname : name to be given to the parameter in the file (string)
%   p     : the value of the parameter (scalar)
%
%   count : number of bytes written
%
% Copyright A. Richards, MIT, 2002
%

c=0;

if size(p)==[1 1],
   % scalar
   c = c + fprintf(fid,['param ' pname ' := %20.15f;\n'],p);
else
   error('Not a scalar')
end

count=c;