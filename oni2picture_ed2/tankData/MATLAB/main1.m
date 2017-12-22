%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%process depth map and ycbcr map
%process wajueji2, frame 182 to 200
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

for k = 1:181 %1:frameNum
    fu_fg_d = imread([fusioned_Fg_d_file,'fusionedForegroundData',int2str(k),'.png']);
    fu_fg_c = imread([fusioned_Fg_c_file,'fusionedForegroundData',int2str(k),'.png']);
    mask_d4c = saveDepthMask(fu_bg_d, fu_fg_d, k, subs_file, fu_fg_c);
    mask_c4d = saveColorMask(fu_bg_c, fu_fg_c);
    %======process ycbcr map======%
    mask_gbf_c = extractTank_color_ycbcr_Func(fu_bg_c, fu_fg_c, k, mask_d4c, mask_c4d, ones(size(fu_fg_d)));
    %======process depth map======%
    mask = extractTankFunc(fu_bg_d, fu_fg_d, fu_fg_c, k, mask_gbf_c, mask_d_c_file, mask_d4c, ones(size(fu_fg_d)));    
    %======depth recovery======%  
    depthRecovery(mask, fu_fg_c, fu_fg_d, k, result_file);
end




