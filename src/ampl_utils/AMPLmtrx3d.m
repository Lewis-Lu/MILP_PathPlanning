function count=AMPLmtrx3d(fid,pname,p,varargin)
%
%   function count=AMPLmatrix(fid,pname,p,[rowheads,colheads,zheads])
%
% Write matrix of floats to AMPL data file
%
%   fid   : file handle of the data file from 'fopen' 
%   pname : name to be given to the parameter in the file (string)
%   p     : the value of the parameter (3D matrix)
%
%   rowheads (optional) : row indices
%   colheads (optional) : column indices
%   zheads   (optional) : indices for third dimension
%
%   If the matrix is M x N x P, rowheads should be 1 x N, colheads
%   should be 1 x M and zheads should be 1 x P. If omitted, rows 
%   are labeled 1 to N, columns 1 to M ad third dimension 1 to P. 
%   This facility allows zero-based indices to be
%   used in the AMPL parameter, or any other index system
%
%   count : number of bytes written
%
% Copyright A. Richards, MIT, 2002
%

% default indices
rowind=[1:size(p,1)];
colind=[1:size(p,2)];
zind=[1:size(p,3)];

% get specified indices
if nargin>3,
  rowind = varargin{1};
end
if nargin>4,
  colind = varargin{2};
end
if nargin>5,
  zind = varargin{3};
end


s=size(p);
c=0;
if size(s,2)<2,
   error('Not 3d matrix')
else
   % 3D matrix
   c = c + fprintf(fid,['param ' pname ' := \n']);
   w = size(p,2);
   h = size(p,1);
   d = size(p,3);
   for k=[1:(d-1)],
      c = c + fprintf(fid,['[*,*,' int2str(zind(k)) '] : ']);
      for j=[1:w],
         c = c + fprintf(fid,'%21.0f',colind(j));
      end
      c = c + fprintf(fid,'\t:=\n');
      for i=[1:(h-1)],
         c = c + fprintf(fid,'\t%4.0f',rowind(i));
         for j=[1:w],
            c = c + fprintf(fid,'%21.14f',p(i,j,k));
         end
         c = c + fprintf(fid,'\n');
      end
      c = c + fprintf(fid,'\t%4.0f',rowind(h));
      for j=[1:w],
         c = c + fprintf(fid,'%21.14f',p(h,j,k));
      end
      c = c + fprintf(fid,'\n');
   end
   k = d;
   c = c + fprintf(fid,['[*,*,' int2str(zind(k)) '] : ']);
   for j=[1:w],
      c = c + fprintf(fid,'%21.0f',colind(j));
   end
   c = c + fprintf(fid,'\t:=\n');
   for i=[1:(h-1)],
      c = c + fprintf(fid,'\t%4.0f',rowind(i));
      for j=[1:w],
         c = c + fprintf(fid,'%21.14f',p(i,j,k));
      end
      c = c + fprintf(fid,'\n');
   end
   c = c + fprintf(fid,'\t%4.0f',rowind(h));
   for j=[1:w],
      c = c + fprintf(fid,'%21.14f',p(h,j,k));
   end
   c = c + fprintf(fid,'\t;\n');
end

count=c;