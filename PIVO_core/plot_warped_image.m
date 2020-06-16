function plot_warped_image(imageLastShow, depthLastShow, imageCurShow, T_21, K)

% double data type
imageLastShow = double(imageLastShow);
depthLastShow = double(depthLastShow);
imageCurShow = double(imageCurShow);


% warp imageCurShow
imageLastShow_warped = warpEntireImage_mex(depthLastShow, imageCurShow, T_21, K);


% plot imageCur from camera and warped calculation
imshow(uint8(imageLastShow)); hold on;
h_warped = imshow(imageLastShow_warped, []); hold off;
[imageHeight, imageWidth] = size(imageLastShow_warped);
blockSize = 100;
P = ceil(imageHeight/blockSize);
Q = ceil(imageWidth/blockSize);
alpha = checkerboard(blockSize, P, Q) > 0;
alpha = alpha(1:imageHeight, 1:imageWidth);
set(h_warped, 'AlphaData', alpha);


end