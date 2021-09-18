function count=AMPLcomment(fid,st)
%
%   function count=AMPLcomment(fid,st)
%
% Write comment to AMPL data file
%
%   fid : file handle of the data file from 'fopen'
%   st  : string to write as comment
%
%   count : number of bytes written
%
% Copyright A. Richards, MIT, 2002
%

c1=fprintf(fid,['# ' st]);
c2=fprintf(fid,'\n');

count=c1+c2;