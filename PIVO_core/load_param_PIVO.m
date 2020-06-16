function [optsPIVO] = load_param_PIVO
% Project:   Patch-based illumination-variant DVO
% Function: load_param_PIVO
%
% Description:
%   get the initial parameters with respect to the algorithm
%
% Example:
%   OUTPUT:
%   optsPIVO: options for PIVO process like below
%
%   INPUT:
%
%
% NOTE:
%     The parameters below are initialized as the TUM paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-01-25: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% define the user-defined parameters
optsPIVO.numHeightBlock = 4;
optsPIVO.numWidthBlock = 4;
optsPIVO.numPtsInSingleBucket = 1;

optsPIVO.minDepth = 0.5;
optsPIVO.thresPlane = 0.02;

optsPIVO.maxPyramidLevel = 5;
optsPIVO.minPyramidLevel = 2;

optsPIVO.winSize = 91;
optsPIVO.patchWinSize = zeros(1, optsPIVO.maxPyramidLevel);
for k = 1:optsPIVO.maxPyramidLevel
    optsPIVO.patchWinSize(k) = floor(optsPIVO.winSize/(2^(k))) * 2 + 1;
end


end