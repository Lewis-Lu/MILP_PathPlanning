function mpcEpisodePlotting(History, Env)
% This function is to plot the batches history in the environment of the episode
%
% function mpcEpisodePlotting(History)
% 
% Version 1.0 : Lu, Hong, 13 Aug 2021
% Email: hlu39@sheffield.ac.uk
% Last Modified: 25 Aug 2021

Pos = History.POS;
Vel = History.Vel;

pos_final = Env.pos_final;


numBatch = numel(Pos);

for i = 1:numBatch
    
end

end