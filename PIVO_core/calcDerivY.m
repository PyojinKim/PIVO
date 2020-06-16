function [ResultY] = calcDerivY(inputImg)
% Project:   Patch-based illumination-variant DVO
% Functions: calcDerivY
%
% Description:
%   Calculate the derivative value of the input image in Y direction;
%
% Example:
%   OUTPUT:
%   ResultY: the derivative image in Y direction from SrcImg
%
%   INPUT:
%   inputImg: the input source image
%
% NOTE:
%      The input image must have only one channel
%      Use this function like [Image_dy] = CalcDerivativeY(Image);
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

ResultY = -1000*ones(rows,cols);
for y = 1:rows
    for x = 1:cols
        prev = max(y-1,1);
        next = min(y+1,rows);
        
        % remove NaN depth pixels
        if (inputImg(next,x) < 0) || (inputImg(y,x) < 0) || (inputImg(prev,x) < 0)
            ResultY(y,x) = -1000;
        else
            ResultY(y,x) = (inputImg(next,x) - inputImg(prev,x)) * 0.5;
        end
    end
end

end