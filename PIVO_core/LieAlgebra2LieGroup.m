function [T_vo_update] = LieAlgebra2LieGroup(delta_ksi)
% Project:   Stereo Visual Odometry with Sequential Geometric and Photometric Optimization
% Function: LieAlgebra2LieGroup
%
% Description:
%   exponential map from Lie algebra se(3) to Lie group SE(3)
%
% Example:
%   OUTPUT:
%   T_vo_update : rigid body motion matrix (4*4) included in SE(3),
%                       which describe the camera motion between the two successive images
%
%   INPUT:
%   delta_ksi: a 6x1 vector included in se(3) which includes camera motion parameters
%                ( delta_ksi = (v1 v2 v3 w1 w2 w3)' ).
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
% 2014-08-20: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate exponential mapping
T_vo_update = eye(4);
v = delta_ksi(1:3);
w = delta_ksi(4:6);

length_w = sqrt(w.' * w);
Wx = hatOperator(w);
if (length_w < 1e-7)
    R = eye(3) + Wx + 0.5*Wx*Wx;
    V = eye(3) + 0.5*Wx + Wx*Wx/3;
else
    R = eye(3) + (sin(length_w)/length_w)*Wx + ((1-cos(length_w))/length_w^2)*(Wx*Wx);
    V = eye(3) + ((1-cos(length_w))/(length_w^2))*Wx + ((length_w-sin(length_w))/(length_w^3))*(Wx*Wx);
end
t = V*v;

% assign R and t to T_vo_update
T_vo_update(1:3,1:3) = R;
T_vo_update(1:3,4) = t;

end






function [what] = hatOperator(vector)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: TwistMatrix
%
% Description:
%   Get the twist matrix of a vector.
%
% Example:
%   OUTPUT:
%   T: skew-symmetric matrix 3x3
%   T = |0 -v3 v2|
%       |v3 0 -v1|
%       |-v2 v1 0|
%
%   INPUT:
%   Vec: The input vector
%   vector = (v1,v2,v3)
%
% NOTE:
%
% Author: Ethan Zhou
% Email: cavatina@yeah.net
% Website: https://github.com/Ethan-Zhou/Dense-VO
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2014-03-03: Complete
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% make skew-symmtric matrix
w1 = vector(1);
w2 = vector(2);
w3 = vector(3);

what = [0 -w3 w2;w3 0 -w1;-w2 w1 0];
end