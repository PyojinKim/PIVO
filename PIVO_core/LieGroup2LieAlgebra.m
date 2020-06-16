function [delta_ksi] = LieGroup2LieAlgebra(T_vo_update)
% Project:   Stereo Visual Odometry with Sequential Geometric and Photometric Optimization
% Function: LieGroup2LieAlgebra
%
% Description:
%   logarithm map from Lie group SE(3) to Lie algebra se(3)
%
% Example:
%   OUTPUT:
%   delta_ksi: a 6x1 vector included in se(3) which includes camera motion parameters
%                ( delta_ksi = (v1 v2 v3 w1 w2 w3)' ).
%
%   INPUT:
%   T_vo_update : rigid body motion matrix (4*4) included in SE(3),
%                       which describe the camera motion between the two successive images
%
% NOTE:
%      Exponential mapping between Lie algebra and Lie group
%      See equation (5)~(6) in ICRA paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2016-03-20: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate logarithm mapping
R = T_vo_update(1:3,1:3);
length_w = acos((trace(R)-1)/2);
lnR = (length_w/(2*sin(length_w)))*(R-R');

w = [-lnR(2,3) lnR(1,3) -lnR(1,2)];
wx = [0 -w(3) w(2);w(3) 0 -w(1);-w(2) w(1) 0];

if (length_w < 1e-7)
    Vin = eye(3);
    w = [0 0 0];
else
    A = sin(length_w)/length_w;
    B = (1-cos(length_w))/(length_w^2);
    Vin = eye(3)-(1/2)*wx+(1/(length_w^2))*(1-(A/(2*B)))*(wx*wx);
end

% assign v and w to delta_ksi
v = Vin * T_vo_update(1:3,4);
delta_ksi = [v.' w].';


end


