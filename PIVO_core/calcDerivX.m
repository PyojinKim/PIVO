function [ResultX] = calcDerivX(inputImg)
% Project:   Patch-based illumination-variant DVO
% Functions: calcDerivX
%
% Description:
%   Calculate the derivative value of the input image in X direction;
%
% Example:
%   OUTPUT:
%   ResultX: the derivative image in X direction from SrcImg
%
%   INPUT:
%   inputImg: the input source image
%
% NOTE:
%      The input image must have only one channel
%      Use this function like [Image_dx] = CalcDerivativeX(Image);
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-01-25:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

[rows,cols] = size(inputImg);

ResultX = -1000*ones(rows,cols);
for y = 1:rows
    for x = 1:cols
        prev = max(x-1,1);
        next = min(x+1,cols);
        
        % remove NaN depth pixels
        if (inputImg(y,next) < 0) || (inputImg(y,x) < 0) || (inputImg(y,prev) < 0)
            ResultX(y,x) = -1000;
        else
            ResultX(y,x) = (inputImg(y,next) - inputImg(y,prev)) * 0.5;
        end
    end
end

end