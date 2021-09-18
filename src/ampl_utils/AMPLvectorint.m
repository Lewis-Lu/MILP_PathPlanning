function count=AMPLvectorint(fid,pname,p)
%
%   function count=AMPLvector(fid,pname,p)
%
% Write vector of integers to AMPL data file
%
%   fid   : file handle of the data file from 'fopen' 
%   pname : name to be given to the parameter in the file (string)
%   p     : the value of the parameter (vector)
%
%   count : number of bytes written
%
% Copyright A. Richards, MIT, 2002
%

s=size(p);
c=0;

if min(size(p))==1,
   % vector
   l=max(s);
   c = c + fprintf(fid,['param ' pname ' := ']);
   for i=[1:(l-1)],
      c = c + fprintf(fid,'%4.0f %12.0f,',i,p(i));
   end
   c = c + fprintf(fid,'%4.0f %12.0f;\n',l,p(l));
else
   error('Not vector')
end

count=c;