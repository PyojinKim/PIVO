function [T_21_final, illParam_final, meanError_final, numIter] = estimateCameraMotion(I1, D1, featurePts, featureNum, I2, winSize, K, T_21_ini, illParam_ini)


I1dx = calcDerivX(I1);
I1dy = calcDerivY(I1);
I2dx = calcDerivX(I2);
I2dy = calcDerivY(I2);


%% determine image patch (R)

pixelPtsRef = zeros(3, winSize*winSize*featureNum);
depthPtsRef = zeros(1, winSize*winSize*featureNum);
patchIdxRef = zeros(1, winSize*winSize*featureNum);
pixelCnt = 0;
for p = 1:featureNum
    
    % setting starting point ( top-left pixel point of the patch )
    uStart = featurePts(1, p) - ((winSize-1)/2) ;
    vStart = featurePts(2, p) - ((winSize-1)/2) ;
    
    for rows = 0:(winSize-1)
        for cols = 0:(winSize-1)
            % check the validity of the location of the pixel
            if (inImage(uStart+cols, vStart+rows, D1))
                if (D1(vStart+rows, uStart+cols) >= 0.5)
                    pixelCnt = pixelCnt + 1;
                    
                    pixelPtsRef(:, pixelCnt) = [uStart+cols; vStart+rows;1];
                    depthPtsRef(:, pixelCnt) = D1(vStart+rows, uStart+cols);
                    patchIdxRef(1, pixelCnt) = p;
                end
            end
        end
    end
end
pixelPtsRef(:, (pixelCnt+1):end) = [];
depthPtsRef(:, (pixelCnt+1):end) = [];
patchIdxRef(:, (pixelCnt+1):end) = [];


%% update camera motion (Jacobian & residual)

T_21 = T_21_ini;
T_21_Save = T_21;
illParam = illParam_ini;
illParam_Save = illParam;

meanErr = Inf;
iterNum = 150;
meanErrSave = zeros(1, iterNum);
ipRelationsNum = size(pixelPtsRef, 2);
camIntrinsicParam = [K(1,1);K(2,2);K(1,3);K(2,3)];
for iterCount = 1:iterNum
    
    jacobian = zeros(ipRelationsNum, (6+2*featureNum));
    residuals = zeros(ipRelationsNum, 1);
    pixelCnt = 0;
    
    for i = 1:ipRelationsNum
        
        u1 = pixelPtsRef(1, i);
        v1 = pixelPtsRef(2, i);
        
        u1_n = (u1 - K(1,3)) / K(1,1);
        v1_n = (v1 - K(2,3)) / K(2,2);
        
        Z_1 = depthPtsRef(1, i);
        X_1 = u1_n * Z_1;
        Y_1 = v1_n * Z_1;
        
        temp = T_21 * [X_1; Y_1; Z_1; 1];
        u2_hat = round(K(1,1) * (temp(1)/temp(3)) + K(1,3));
        v2_hat = round(K(2,2) * (temp(2)/temp(3)) + K(2,3));
        
        if (inImage(u2_hat, v2_hat, I2))
            
            J_I_I1 = [I1dx(v1, u1), I1dy(v1, u1)];
            J_I_I2 = [I2dx(v2_hat, u2_hat), I2dy(v2_hat, u2_hat)];
            J_I = 0.5 * (J_I_I1 + J_I_I2);
            
            if (~((abs(J_I(1)) <= 0.00001 && abs(J_I(2)) <= 0.00001) || I1(v1, u1) <= 5 || I1(v1, u1) >= 250 || ...
                    I2(v2_hat, u2_hat) <= 5 || I2(v2_hat, u2_hat) >= 250))
                pixelCnt = pixelCnt + 1;
                
                p = patchIdxRef(1, i);
                ctrst = illParam(1, p);
                bias = illParam(2, p);
                I1intensity = I1(v1, u1);
                I2intensity = I2(v2_hat, u2_hat);
                
                % Jacobian vector
                T_21_3x4 = T_21(1:3,:);
                du2hatdxi = du2hat_dxi([X_1;Y_1;Z_1;T_21_3x4(:);camIntrinsicParam]);
                dv2hatdxi = dv2hat_dxi([X_1;Y_1;Z_1;T_21_3x4(:);camIntrinsicParam]);
                J_w = [du2hatdxi; dv2hatdxi];
                jacobian(pixelCnt, 1:6) = ctrst * (J_I) * (J_w);
                jacobian(pixelCnt, (2*p+5)) = I2intensity;
                jacobian(pixelCnt, (2*p+6)) = 1;
                
                % residual vector
                residuals(pixelCnt) = (ctrst * I2intensity + bias) - I1intensity;
            end
        end
    end
    jacobian((pixelCnt+1):end, :) = [];
    residuals((pixelCnt+1):end) = [];
    
    % weight vector
    [estiSigma]= tDistriScaleEstimator(5.0, 5, residuals);
    weights = tDistriWeightFtn(residuals, estiSigma, 5);
    
    % average cost function
    meanErrLast = meanErr;
    meanErr = mean(weights.*(residuals.^2));
    meanErrSave(iterCount) = meanErr;
    
    
    % stop condition
    if (meanErr > meanErrLast)
        fprintf('error increases...at k:%f \n',iterCount);
        break;
    else
        % next iteration
        T_21_Save = T_21;
        illParam_Save = illParam;
        
        % delta z
        invalidPatchIdx = getInvalidPatchIdx(jacobian, featureNum, winSize);
        deltaZ = calcDeltaZ(jacobian, residuals, weights, featureNum, invalidPatchIdx, 'GN');
        
        T_21 = T_21 * LieAlgebra2LieGroup(deltaZ(1:6));
        for p = 1:featureNum
            illParam(1, p) = illParam(1, p) + deltaZ(2*p+5);
            illParam(2, p) = illParam(2, p) + deltaZ(2*p+6);
        end
    end
end

% record T_21, illParam, and error value at minimum cost value
T_21_final = T_21_Save;
illParam_final = illParam_Save;
meanError_final = meanErrSave(iterCount-1);
numIter = iterCount;


end

