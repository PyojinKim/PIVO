clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% basic setup for PIVO

% choose the experiment case
% ICL NUIM dataset (1~8)
expCase = 1;

% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run PIVO
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run PIVO
toSave = 1;


setupParams_ICL_NUIM;


% load ICL NUIM dataset data
rawICLNUIMdataset = rawICLNUIMdataset_load(datasetPath);


% camera calibration parameters
[ICLNUIMdataset] = getSyncTUMRGBDdataset(rawICLNUIMdataset, imInit, M);
optsPIVO = load_param_PIVO;
cam = initialize_cam_ICL_NUIM(ICLNUIMdataset, optsPIVO.maxPyramidLevel);


%% load ground truth data

% ground truth trajectory in ICL NUIM dataset
R_gc_true = zeros(3,3,M);
p_gc_true = zeros(3,M);
T_gc_true = cell(1,M);
for k = 1:M
    % camera body frame
    R_gc_true(:,:,k) = q2r(ICLNUIMdataset.vicon.q_gc_Sync(:,k));
    p_gc_true(:,k) = ICLNUIMdataset.vicon.p_gc_Sync(:,k);
    T_gc_true{k} = [ R_gc_true(:,:,k), p_gc_true(:,k);
        zeros(1,3),           1; ];
end
if (toVisualize)
    figure; hold on; axis equal;
    L = 0.1; % coordinate axis length
    A = [0 0 0 1; L 0 0 1; 0 0 0 1; 0 L 0 1; 0 0 0 1; 0 0 L 1]';
    
    for k = 1:10:M
        T = T_gc_true{k};
        B = T * A;
        plot3(B(1,1:2),B(2,1:2),B(3,1:2),'-r','LineWidth',1); % x: red
        plot3(B(1,3:4),B(2,3:4),B(3,3:4),'-g','LineWidth',1); % y: green
        plot3(B(1,5:6),B(2,5:6),B(3,5:6),'-b','LineWidth',1); % z: blue
    end
    plot3(p_gc_true(1,:),p_gc_true(2,:),p_gc_true(3,:),'k','LineWidth',2);
    
    title('ground truth trajectory of cam0 frame')
    xlabel('x'); ylabel('y'); zlabel('z');
end


% generate ground truth trajectory in vector form
stateTrue = zeros(6,M);
stateTrue(1:3,:) = p_gc_true;
for k = 1:size(p_gc_true,2)
    [yaw, pitch, roll] = dcm2angle(R_gc_true(:,:,k));
    stateTrue(4:6,k) = [roll; pitch; yaw];
end


%% main PIVO part

% initialize variables for PIVO
T_gc_PIVO = cell(1,M);
illParam_PIVO = cell(1,M);
meanErrorRec = zeros(1,M);
numIterRec = cell(1,M);


% rigid body transformation of the first frame
T_gc_current = T_gc_true{1};
T_gc_PIVO{1} = T_gc_current;


% make figures to visualize current status
if (toVisualize)
    % create figure
    h=figure(10);
    set(h,'Color',[1 1 1]);
    set(h,'Units','pixels','Position',[150 600 1700 400]);
    ha1 = axes('Position',[0.025,0.1 , 0.3,0.8]);
    axis off;
    ha2 = axes('Position',[0.350,0.1 , 0.3,0.8]);
    axis off;
    ha3 = axes('Position',[0.675,0.1 , 0.3,0.8]);
    axis equal; grid on; hold on;
end


% do PIVO
for imgIdx = 2:M
    
    % keyframe image
    imageLast = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, (imgIdx-1), 'rgb');
    depthLast = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, (imgIdx-1), 'depth');
    [imageLastPyramid, depthLastPyramid, featuresPyramid] = getPatchImgPyramidinMAV(imageLast, depthLast, optsPIVO);
    featureNum = size(featuresPyramid{1},2);
    
    
    % current image
    imageCur = getImgInTUMRGBDdataset(datasetPath, ICLNUIMdataset, cam, imgIdx, 'gray');
    [imageCurPyramid,~] = getImgPyramid(imageCur, eye(32), optsPIVO.maxPyramidLevel);
    
    
    % frame to frame motion estimation
    T_21_ini = eye(4);
    illParam_ini = [ones(1,featureNum); zeros(1,featureNum)];
    
    T_21_Rec = T_21_ini;
    illParam_Rec = illParam_ini;
    numIterPyramid = zeros(5,1);
    
    for L = (optsPIVO.maxPyramidLevel):-1:(optsPIVO.minPyramidLevel)
        
        % assign current pyramid
        I1 = imageLastPyramid{L};
        D1 = depthLastPyramid{L};
        featurePts = featuresPyramid{L};
        I2 = imageCurPyramid{L};
        winSize = optsPIVO.patchWinSize(L);
        K = cam.K_pyramid(:,:,L);
        
        [T_21_Rec, illParam_Rec, meanError, numIter] = estimateCameraMotion_mex(I1, D1, featurePts, featureNum, I2, winSize, K, T_21_Rec, illParam_Rec);
        numIterPyramid(L) = numIter;
    end
    
    T_21_final = T_21_Rec;
    illParam_final = illParam_Rec;
    
    
    % for next iteration
    illParam_PIVO{imgIdx} = illParam_final;
    meanErrorRec(imgIdx) = meanError;
    numIterRec{imgIdx} = numIterPyramid;
    meanError
    
    
    %% 3. update 6 DoF camera pose and visualization
    
    if (imgIdx >= 2)
        % update current camera pose
        T_gc_current = T_gc_current * inv(T_21_final);
        T_gc_PIVO{imgIdx} = T_gc_current;
        
        
        % visualize current status
        plots_ICL_NUIM;
    end
    
    
end

% convert camera pose representation
stateEsti_PIVO = zeros(6, M);
R_gc_PIVO = zeros(3,3,M);
for k = 1:M
    R_gc_PIVO(:,:,k) = T_gc_PIVO{k}(1:3,1:3);
    stateEsti_PIVO(1:3,k) = T_gc_PIVO{k}(1:3,4);
    [yaw, pitch, roll] = dcm2angle(R_gc_PIVO(:,:,k));
    stateEsti_PIVO(4:6,k) = [roll; pitch; yaw];
end


%% plot error metric value (RPE, ATE)

% 1) PIVO motion estimation trajectory results
figure;
plot3(stateTrue(1,:),stateTrue(2,:),stateTrue(3,:),'k','LineWidth',2); hold on; grid on;
plot3(stateEsti_PIVO(1,:),stateEsti_PIVO(2,:),stateEsti_PIVO(3,:),'r','LineWidth',2);
legend('True','PIVO Matlab'); plot_inertial_frame(0.5); axis equal; view(-158, 38);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% 2) calculate RPE / ATE / EPE error metric for PIVO
[RPE_RMSE_PIVO,RPE_PIVO] = calcRPE(T_gc_PIVO, T_gc_true, 30, 'RMSE');
[ATE_RMSE_PIVO,ATE_PIVO] = calcATE(T_gc_PIVO, T_gc_true, 'RMSE');
[EPE_PIVO,lengthDataset] = calcEPE(stateEsti_PIVO, stateTrue);
fprintf('*******************************\n')
fprintf('RMSE of RPE [drift m/s] : %f \n' , RPE_RMSE_PIVO);
fprintf('RMSE of ATE [m] : %f \n' , ATE_RMSE_PIVO);
fprintf('EPE [%%] : %f \n' , EPE_PIVO);
fprintf('Total traveled distance [m] : %f \n' , lengthDataset);
fprintf('*******************************\n')


% 3) draw figures for RPE, ATE
figure;
plot(RPE_PIVO); grid on;
xlabel('Time [ # of images]','fontsize',10); ylabel('PIVO drift error [m/s]','fontsize',10);
title('Relative Pose Error (RPE) versus time index','fontsize',15)

figure;
plot(ATE_PIVO); grid on;
xlabel('Time [ # of images]','fontsize',10); ylabel('PIVO absolute trajectory error [m]','fontsize',10);
title('Absolute Trajectory Error (ATE) versus time index','fontsize',15)


% 4) each frame's residual result of PIVO
figure;
plot(meanErrorRec,'r','LineWidth',1); grid on;
title('PIVO cost value of PIVO','fontsize',10);
xlabel('The index of image frame','fontsize',10); ylabel('Residual','fontsize',10);


% 5) total traveling distance of this dataset by PIVO
dDistance = diff(stateEsti_PIVO(1:3,:),1,2);
lengthDataset = sum(sqrt(sum(dDistance.^2,1)));
fprintf('*******************************\n')
fprintf('Total traveled distance [m] : %f \n' , lengthDataset);
fprintf('*******************************\n')


%% save the experiment data for AURO 2019

if (toSave)
    save([SaveDir '/PIVO.mat']);
end



