function [invalidPatchIdx] = getInvalidPatchIdx(Je, numPatch, winSize)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: getInvalidPatchIdx
%
% Description:
%   get the index of invalid patch for only considering the valid patches
%   invalid patch will have less than 20% of the total number of pixel
%
% Example:
%   OUTPUT:
%   invalidPatchIdx: the index of invalid patch
%
%
%   INPUT:
%   Je: Net Jacobian matrix ( (?) x (6+2m) )
%   numPatch: the number of the whole patches
%   winSize: the window size of the patch at current pyramid level
%
% NOTE:
%       The input of the function Je, We, re are subsets of the original J,
%       W, r, which exclude the ones whose values are invalid for various
%       reason. See also equation (21) in ICRA paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-02-12: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% set parameters
numPixelInEachPatch = zeros(1, numPatch);
minNumPixelInEachPatch = ceil(0.2*winSize*winSize); % the minimum number of pixels in each patch


% count the number of pixels in each patch
for patchIdx=1:numPatch
    numPixelInEachPatch(1,patchIdx) = sum(Je(:,(2*patchIdx+6)));
end


% extract the invalid patch index
invalidPatchIdx = [];
for patchIdx = 1:numPatch
    if (numPixelInEachPatch(1,patchIdx) <= minNumPixelInEachPatch)
        invalidPatchIdx = [invalidPatchIdx,patchIdx];
    end
end


end

