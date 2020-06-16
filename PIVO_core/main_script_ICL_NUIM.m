clc;
close all;
clear variables; %clear classes;
rand('state',0); % rand('state',sum(100*clock));
dbstop if error;


%% basic setup for PIVO

% choose the experiment case
% ICSL dataset (1~4)
expCase = 1;

% are figures drawn?
% 1 : yes, draw figures to see current status
% 0 : no, just run PIVO
toVisualize = 1;

% are data results saved?
% 1 : yes, save the variables and results
% 0 : no, just run PIVO
toSave = 1;


setupParams_ICSL;


% load ICSL dataset data
imLeftDir = [ datasetPath '/mav0/cam0/data/' ];
imRightDir = [ datasetPath '/mav0/cam1/data/' ];


% camera calibration parameters
ICSLdataset = dir(imLeftDir);
ICSLdataset(1:2) = [];
ICSLdataset = ICSLdataset(imInit:(imInit+M-1));
opts = load_param_PIVO;
cam = initialize_cam_ICSL(opts.maxPyramidLevel);


%% main PIVO part


% initialize variables for PIVO
T_gc_PIVO = cell(1,M);
illParam_PIVO = cell(1,M);
meanErrorRec = zeros(1,M);
numIterRec = cell(1,M);


% rigid body transformation of the first frame
T_gc_current = eye(4);
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
    imageLast = getImgInICSLdataset(imLeftDir, imRightDir, ICSLdataset, cam, (imgIdx-1), 'gray');
    depthLast = getImgInICSLdataset(imLeftDir, imRightDir, ICSLdataset, cam, (imgIdx-1), 'depth');
    [imageLastPyramid, depthLastPyramid, featuresPyramid] = getPatchImgPyramidinMAV(imageLast, depthLast, opts);
    featureNum = size(featuresPyramid{1}, 2);
    
    
    % current image
    imageCur = getImgInICSLdataset(imLeftDir, imRightDir, ICSLdataset, cam, imgIdx, 'gray');
    [imageCurPyramid,~] = getImgPyramid(imageCur, eye(32), opts.maxPyramidLevel);
    
    
    % frame to frame motion estimation
    T_21_ini = eye(4);
    illParam_ini = [ones(1, featureNum); zeros(1, featureNum)];
    
    T_21_Rec = T_21_ini;
    illParam_Rec = illParam_ini;
    numIterPyramid = zeros(5,1);
    
    for L = (opts.maxPyramidLevel):-1:(opts.minPyramidLevel)
        
        % assign current pyramid
        I1 = imageLastPyramid{L};
        D1 = depthLastPyramid{L};
        featurePts = featuresPyramid{L};
        I2 = imageCurPyramid{L};
        winSize = opts.patchWinSize(L);
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
        plots_ICSL;
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
plot3(stateEsti_PIVO(1,:),stateEsti_PIVO(2,:),stateEsti_PIVO(3,:),'r','LineWidth',2); hold on; grid on;
legend('PIVO Matlab'); plot_inertial_frame(0.5); axis equal; view(-158, 38);
xlabel('x [m]','fontsize',10); ylabel('y [m]','fontsize',10); zlabel('z [m]','fontsize',10); hold off;


% 2) each frame's residual result of PIVO
figure;
plot(meanErrorRec,'r','LineWidth',1); grid on;
title('PIVO cost value of PIVO','fontsize',10);
xlabel('The index of image frame','fontsize',10); ylabel('Residual','fontsize',10);


% 3) total traveling distance of this dataset by PIVO
dDistance = diff(stateEsti_PIVO(1:3,:),1,2);
lengthDataset = sum(sqrt(sum(dDistance.^2,1)));
fprintf('*******************************\n')
fprintf('Total traveled distance [m] : %f \n' , lengthDataset);
fprintf('*******************************\n')


%% save the experiment data for TRO 2017

if (toSave)
    save([SaveDir '/PIVO_vopp_discussion.mat']);
end



