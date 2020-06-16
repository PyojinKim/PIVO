function [sigma]= tDistriScaleEstimator(initialSigma,tDistriDof,residual)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: tDistriScaleEstimator
%
% Description:
%   Get the sqrt(variance) of the T distribution. ( based on equation (4.32) )
%
% Example:
%   OUTPUT:
%   sigma: estimated scale of residual distribution
%
%   INPUT:
%   initialSigma: Initial sigma, which is used in scale estimation iteratively.
%   tDistriDof: default degree of freedom, which is used in scale estimation iteratively.
%   residual: Residual vector ( (mn) x 1 )
%
% NOTE:
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2015-04-06: ing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

% set parameters
lambda = 1/(initialSigma^2); % Lambda = 1/(sigma^2)

for i=1:length(residual)
    % initial value
    initialLambda = lambda;
    
    % estimate scale parameter
    sigmaSquared = mean( ((tDistriDof+1) ./ (tDistriDof + initialLambda*(residual.^2))) .* (residual.^2) );
    
    % transform
    lambda = 1/sigmaSquared;
    
    % exit condition
    if abs(lambda - initialLambda) <= 1e-6
        break;
    end
end

% retransform results
sigma = sqrt(1/lambda);

% exception handling
if sigma < 0.001
    sigma = 0.001;
end

end



