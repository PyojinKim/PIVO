function [I2warp] = warpEntireImage(D1, I2, T_21, K)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: warpEntireImage
%
% Description:
%   warp the image I2 -> I2warp
%
% Example:
%   OUTPUT:
%   I2warp: the warped image from D1 and I2 value
%
%   INPUT:
%   D1: reference depth image (with double type)
%   I2: next gray image (with double type)
%   T_21: [4x4] camera motion matrix. [P_body] = T_21 * [P_inertial]
%   K: matrix form of camera intrinsic parameter (3x3)
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-01-25: ing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% pre-defined variables
imageHeight = size(D1, 1);
imageWidth = size(D1, 2);
I2warp = zeros(imageHeight, imageWidth);
pixelPtsRef = zeros(3, imageHeight*imageWidth);
depthPtsRef = zeros(3, imageHeight*imageWidth);


% corresponding I2 image
D1(D1 == 0) = NaN;
pixelCnt = 0;
for v = 1:imageHeight
    for u = 1:imageWidth;
        if (isnan(D1(v, u)) ~= 1)
            pixelCnt = pixelCnt + 1;
            
            pixelPtsRef(:, pixelCnt) = [u; v; 1];
            depthPtsRef(:, pixelCnt) = D1(v, u) * ones(3,1);
        end
    end
end
pixelPtsRef(:, (pixelCnt+1):end) = [];
depthPtsRef(:, (pixelCnt+1):end) = [];


% I1 frame
normPtsRef = inv(K) * pixelPtsRef;
pointCloudRef = [normPtsRef .* depthPtsRef;
    ones(1, size(normPtsRef,2))];


% I2 frame
pointCloudNext = T_21 * pointCloudRef;
normPtsNext = pointCloudNext([1 2 3], :) ./ pointCloudNext([3 3 3],:);
pixelPtsNext = round(K * normPtsNext);

for k = 1:size(pixelPtsNext,2)
    x = pixelPtsNext(1, k);
    y = pixelPtsNext(2, k);
    if (inImage(x, y, I2warp))
        u = pixelPtsRef(1, k);
        v = pixelPtsRef(2, k);
        I2warp(v,u) = I2(y,x);
    end
end
I2warp(I2warp == 0) = NaN;


end