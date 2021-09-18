function videoMaker(filename, fps, figureList)
% This function is video Maker
% no return(s).
%
% function videoMaker(filename, fps, figureList)
% 
% Version 1.0 : Lu, Hong, 15 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 15 Aug 2021
    wObj = VideoWriter(filename);
    wObj.FrameRate = fps;
    
    open(wObj);
    
    for i = 1:length(figureList)
        fr = im2frame(imread(['video/img/' figureList{i}]));
        writeVideo(wObj,fr);
    end
    
    close(wObj);
end