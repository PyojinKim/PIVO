function result = inImage(u, v, SrcImg)
% Project:   Patch-based illumination-variant DVO
% Function: inImage
%
% Description:
%   Judge whether the point is in the image plane after warping process
%
% Example:
%   OUTPUT:
%   result: 0 if out
%             1 if in
%
%   INPUT:
%   u,v: The u,v coordinate of the point (in pixel image plane).
%   SrcImg: The input source image.
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-02-25: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
[rows, cols] = size(SrcImg);

if(u >= 1 && u <=cols && v >=1 && v <= rows)
    result = 1;
else
    result = 0;
end

end

