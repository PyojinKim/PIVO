function [weights] = tDistriWeightFtn(residual,tDistriSigma,tDistriDof)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: tDistriWeightFtn
%
% Description:
%   Get the weight for every element of residual, which will be used in
%   re-weighted least square algorithm. ( see equation (23) in ICRA paper )
%
% Example:
%   OUTPUT:
%   weight: weight of each pixel based on t-distribution function ( (numValidPixels) x 1 )
%
%   INPUT:
%   residual: Residual vector ( (numValidPixels) x 1 )
%   sigma: estimated scale of residual distribution
%   dof: degree of freedom parameter used in t-distribution
%
% NOTE:
%      see TUM paper 'Odometry from RGB-D Cameras for Autonomous Quadropters',5.3.8.
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-04-08: ing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% initial values
sigma_t = tDistriSigma * ones(size(residual));

% compute weight function
weights = (tDistriDof + 1) ./ (tDistriDof + ((residual./sigma_t).^2));

end

