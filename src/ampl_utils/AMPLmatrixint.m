function count=AMPLmatrixint(fid,pname,p)
%
%   function count=AMPLmatrix(fid,pname,p)
%
% Write matrix of integers to AMPL data file
%
%   fid   : file handle of the data file from 'fopen' 
%   pname : name to be given to the parameter in the file (string)
%   p     : the value of the parameter (matrix)
%
%   count : number of bytes written
%
% Copyright A. Richards, MIT, 2002
%

s=size(p);
c=0;

if size(s,2)>2,
   error('Not matrix')
else
   % matrix
   w = size(p,2);
   c = c + fprintf(fid,['param ' pname ' : ']);
   for j=[1:w],
      c = c + fprintf(fid,'%12.0f',j);
   end
   c = c + fprintf(fid,'\t:=\n');
   h = size(p,1);
   for i=[1:(h-1)],
      c = c + fprintf(fid,'\t%4.0f',i);
      for j=[1:w],
         c = c + fprintf(fid,'%12.0f',p(i,j));
      end
      c = c + fprintf(fid,'\n');
   end
   c = c + fprintf(fid,'\t%4.0f',h);
   for j=[1:w],
      c = c + fprintf(fid,'%12.0f',p(h,j));
   end
   c = c + fprintf(fid,'\t;\n');
end

count=c;