function [I1RefPyr,D1RefPyr,keyPtsPyr] = getPatchImgPyramidinMAV(I1Ref,D1Ref,optsPIVO)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: getPatchImgPyramidinMAV
%
% Description:
%   read input and depth images, and generate their pyramid images & keypoints
%   But, based on RANSAC with plane model, all selected patches are on the plane
%
% Example:
%   OUTPUT :
%   I1RefPyr: gray image pyramid of the selected reference gray image
%   D1RefPyr: depth image pyramid with respect to I1RefPyr
%   keyPtsPyr: the remain keypoints which are the center of the plane patch
%                  (only plane patches are saved)
%
%   INPUT :
%   I1Ref: A rgb or gray input image (with uint8 type)
%   D1Ref: A depth input image (with double type)
%   optsPIVO: options for PIVO process like below
%
% NOTE:
%      The input images are converted to plane-patched partial image
%      with Pyramid generation.
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2016-02-25 : Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%


% load gray and depth image
if size(I1Ref,3) == 3
    grayImg = double(rgb2gray(I1Ref));
else
    grayImg = double(I1Ref);
end
depthImg = double(D1Ref);


%% 1. gray and depth image pyramid

I1RefPyr = cell(1,optsPIVO.maxPyramidLevel);
D1RefPyr = cell(1,optsPIVO.maxPyramidLevel);

I1RefPyr{1} = grayImg;
D1RefPyr{1} = depthImg;

for i = 2:optsPIVO.maxPyramidLevel
    I1RefPyr{i} = downsampleImage(I1RefPyr{i-1}); % impyramid(I1RefPyr{i-1},'reduce');
    D1RefPyr{i} = downsampleDepth(D1RefPyr{i-1}); % impyramid(D1RefPyr{i-1},'reduce');
end


%% 2. manual keypoints detection (optional)

% for EuRoC MAV dataset
[rows,cols] = size(grayImg);
rowsUnit = round(rows/4);
colsUnit = round(cols/5);
keyPtsAll = zeros(2,20);

ptsCnt = 0;
for i = 1:4
    for j = 1:5
        ptsCnt = ptsCnt + 1;
        
        keyPtsAll(1,ptsCnt) = round((colsUnit*j) - (colsUnit/2) + 1);    % u pixel
        keyPtsAll(2,ptsCnt) = round((rowsUnit*i) - (rowsUnit/2) + 1);  % v pixel
    end
end

keyPtsPyr = cell(1, optsPIVO.maxPyramidLevel);
keyPtsPyr{1} = keyPtsAll;

for i = 2:optsPIVO.maxPyramidLevel
    keyPtsPyr{i} = ceil(keyPtsPyr{1} ./ (2^(i-1)));
end


end
