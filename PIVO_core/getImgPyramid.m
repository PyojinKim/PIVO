function [I2Pyr,D2Pyr] = getImgPyramid(I2, D2, maxPyramidLevel)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: getImgPyramid
%
% Description:
%   Read input image and generate their pyramid images
%
% Example:
%   OUTPUT :
%   I2Pyr: Gray image pyramid of the input image
%   D2Pyr: Depth image pyramid with respect to I2Pyr
%
%   INPUT :
%   I2: A rgb or gray input image (with uint8 type)
%   D2: A depth input image (with uint16 type)
%   maxPyramidLevel: options for PIVO process like below
%
% NOTE:
%
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2016-04-05 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%% Read Input RGB (or Gray) image frame
%  and convert them from [uint8], [uint16] to [double] data type

if size(I2,3) == 3
    grayImg = double(rgb2gray(I2));
else
    grayImg = double(I2);
end

% recover to the real metric depth value [m]
depthImg = double(D2);


%% Generate the gray image pyramids

% make the empty cells
I2Pyr = cell(1, maxPyramidLevel);
D2Pyr = cell(1, maxPyramidLevel);

% initialize
I2Pyr{1} = grayImg;
D2Pyr{1} = depthImg;

% perform pyramid reduction
for i = 2:maxPyramidLevel
    I2Pyr{i} = downsampleImage(I2Pyr{i-1});    % downsampleImage outperform than impyramid ftn in MATLAB
    D2Pyr{i} = downsampleDepth(D2Pyr{i-1});   % impyramid(D1RefPyr{i-1},'reduce');
end


end
