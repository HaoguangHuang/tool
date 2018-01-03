%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%process depth map and ycbcr map
%process wajueji2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clear all; close all;
fusioned_Bg_d_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/depth/fusionedBackgroundData/fusionedBackgroundData.png';
fusioned_Fg_d_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/depth/fusionedForegroundData/';
fusioned_Bg_c_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/color/fusionedBackgroundData/fusionedBackgroundData.png';
fusioned_Fg_c_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/color/fusionedForegroundData/';
frameNum = 100;
  
%======substract foreground from background======%
subs_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/intensityMask/'; %save depth/ycbcr map after substracting between fg and bg
mask_d_c_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/182_200/';  %save final mask
result_file = '/home/hhg/Downloads/dataSet/Wajueji_2/processedData/182_200/result/';     %save ycbcr/depth map after segmenting by final mask and depthRecovery
fu_bg_d = imread(fusioned_Bg_d_file);
fu_bg_c = imread(fusioned_Bg_c_file);
global debug_mode; debug_mode = 1;
global gt;         gt = zeros(size(fu_bg_d)); 

% gaijin = [19:27,29:35,38:40,44:47,49,50,53,54,60,61,63,64,65,67,68,85,86,87,96:105,107:111,113,118,119,122,...
%     128,129,140,141,142,144,145,148];
% dan du chu li:45,46
gaijin = [64,65,67,68,85,86,87,96:105,107:111,113,118,119,122,...
    128,129,140,141,142,144,145,148];
 
for k = 64 %1:frameNum
    fu_fg_d = imread([fusioned_Fg_d_file,'fusionedForegroundData',int2str(k),'.png']);
    fu_fg_c = imread([fusioned_Fg_c_file,'fusionedForegroundData',int2str(k),'.png']);
    mask_d4c = saveDepthMask(fu_bg_d, fu_fg_d, k, subs_file, fu_fg_c);
    mask_c4d = saveColorMask(fu_bg_c, fu_fg_c);
    %======process ycbcr map======%
    mask_gbf_c = extractTank_color_ycbcr_Func(fu_bg_c, fu_fg_c, k, mask_d4c, mask_c4d, ones(size(fu_fg_d)));
    %======process depth map======%
    mask = extractTankFunc(fu_bg_d, fu_fg_d, fu_fg_c, k, mask_gbf_c, mask_d_c_file, mask_d4c, ones(size(fu_fg_d)));    
    
    I(:,:,1) = mat2gray(mask)*255;
    I(:,:,2) = mat2gray(fu_fg_c(:,:,1));
    I(:,:,3) = zeros(size(mask));
    figure(20),imshow(I),title(sprintf('frame\\_%d, output',k));
    
    figure(21),imshow(fu_fg_c);title(sprintf('frame\\_%d, rgb',k))
    imwrite(mask,sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/houqi_v2/mask_%d.png',k));
    imwrite(uint8(mat2gray(mask)*255),sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/houqi_v2/vis/mask_%d.png',k));
    %======depth recovery======%  
%     depthRecovery(mask, fu_fg_c, fu_fg_d, k, result_file);
end




