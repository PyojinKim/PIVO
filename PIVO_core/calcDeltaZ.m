function [deltaZ] = calcDeltaZ(Je, Re, We, numPatch, invalidPatchIdx, method)
% Project:   Patch-based Illumination invariant Visual Odometry (PIVO)
% Function: calcDeltaZ
%
% Description:
%   get the increment of the motion parameter ksi and brightness param.
%   and consider Multi-Patch illumination changes at the same time
%
% Example:
%   OUTPUT:
%   deltaZ: Minimal representation of motion and brightness parameters per patch ( (6+2*m) x 1 )
%
%
%   INPUT:
%   Je: Net Jacobian matrix ( (numValidPixels) x (6+2m) )
%   Re: Net Residual valeus ( (numValidPixels) x 1 )
%   We: Net Weighting vector ( (numValidPixels) x 1 )
%   numPatch: number of patches
%   invalidPatchIdx: The index of invalid patch
%   method: optimization method
%
% NOTE:
%       The input of the function Je, We, re are subsets of the original J,
%       W, r, which exclude the ones whose values are invalid for various
%       reason. See also equation (21) in ICRA paper
%
% Author: Pyojin Kim
% Email: pjinkim1215@gmail.com
% Website:
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% log:
% 2017-04-05: ing
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

%% set parameters

numValidPatch = numPatch - length(invalidPatchIdx);
temp_deltaZ = zeros((6+2*numValidPatch),1); % for mex ftn
deltaZ = zeros((6+2*numPatch),1);                % for mex ftn

%% calculate increment of model parameters

if (~isempty(invalidPatchIdx))
    %% if there exists invalid patch
    
    
    % index of invalid columns
    invalidColIdx = [];
    for i=1:length(invalidPatchIdx)
        invalidColIdx = [invalidColIdx, 2*invalidPatchIdx(i)+5, 2*invalidPatchIdx(i)+6];
    end
    
    
    % extract only valid patches part
    JeValid = Je;
    JeValid(:,invalidColIdx) = [];
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% debug & check part
    if ( size(JeValid,2) ~= (6+2*numValidPatch) )
        error('size(Je_valid,2) ~= (6+2*Num_valid_patch)');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % approximate Hessian matrix ( 2nd derivative )
    H = JeValid.' * (repmat(We,1,size(JeValid,2)) .* JeValid);
    if (strcmp(method, 'GN'))
        % to prevent bad matrix (singular case)
        if (rcond(H) <= 10^-25)
            fprintf('rcond(H) <= 10^-25 : Bad H condition \n');
            H = 10^-13 * eye(size(JeValid,2));
        end
        
        % calculate temp deltaZ
        temp_deltaZ = - H^-1 * JeValid.' * (We .* Re);
    end
    
    
    % convert temp_deltaZ(only with valid patches) to delta_z (with all patches)
    j = 0;
    for i=1:(6+2*numPatch)
        % check the invalid patch index
        invalidCnt = 0;
        for k=1:length(invalidPatchIdx)
            if ( i == (2*invalidPatchIdx(k)+5) )
                invalidCnt = invalidCnt + 1;
            elseif ( i == (2*invalidPatchIdx(k)+6) )
                invalidCnt = invalidCnt + 1;
            end
        end
        
        % skip the invalid patch index
        if ( invalidCnt > 0 )
            continue;
        end
        
        % assign valid patch values
        j = j + 1;
        deltaZ(i) = temp_deltaZ(j);
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% debug & check part
    if ( j ~= (6+2*numValidPatch) )
        error('j ~= (6+2*Num_valid_patch)');
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
else
    %% if there is no invalid patch
    
    
    % approximate Hessian matrix ( 2nd derivative )
    H = Je.' * (repmat(We,1,size(Je,2)) .* Je);
    if (strcmp(method,'GN'))
        % to prevent bad matrix (singular case)
        if (rcond(H) <= 10^-25)
            fprintf('rcond(H) <= 10^-25 : Bad H condition \n');
            H = 10^-13 * eye(size(Je,2));
        end
        
        % calculate delta z
        deltaZ = - H^-1 * Je.' * (We .* Re);
    end
end

end

