%% TEST_MAIN: this script support to test fg/bg segmentation in dataset
%       ShSeq:420-540     

% clear all; close all;
warning('off');   global debug;  debug = 1;  
series = 'ShSeq'; root_dir = ['E:\dataSet\ICAISS\Hybrid_FBS\' series '\'];
frameStart = 425; frameEnd = 540;
%%==================background fusion==================
[c_bg, d_bg] = Extract_BG_From_DataSet([root_dir series]);
save('data.mat','d_bg')
%%=================foreground substraction===============
for k = frameStart:5:frameEnd
if 1
    d_fg = imread([root_dir '\' series '\depthData\depth_' int2str(k) '.png']);
    c_fg = imread([root_dir '\' series '\ycbcrData\ycbcr_color_' int2str(k) '.png']);
    output_file = [root_dir '\' series '\output\'];
    mask_d4c = saveDepthMask(d_bg, d_fg, k, [], c_fg);
    mask_c4d = saveColorMask(c_bg, c_fg, k, []);
    %%======process ycbcr map======%%
    mask_gbf_c = extractTank_color_ycbcr_Func(c_bg, c_fg, k, mask_d4c, mask_c4d);
else
    mask_gbf_c = load('hhg.mat','mask_gbf_c'); mask_gbf_c = mask_gbf_c.mask_gbf_c;
end
    %%======process depth map======%%
    mask = extractTankFunc(d_bg, d_fg, c_fg, k, mask_gbf_c, output_file, mask_d4c);
    %%======compare with gt=======%%
    gt = imread([root_dir '\' series '\groundTruth\gt_' int2str(k) 'BW.bmp']);
    I(:,:,1) = mat2gray(mask)*255; I(:,:,2) = mat2gray(gt)*255; I(:,:,3) = zeros(size(gt));
    figure(111),imshow(uint8(I));title('gt + mask\_result'),drawnow;
    I(:,:,1) = mat2gray(mask_d4c)*255;
    figure(112),imshow(uint8(I));title('gt + mask\_d4c');drawnow;
    I(:,:,1) = mat2gray(mask_c4d)*255;
    figure(113),imshow(uint8(I)),title('gt + mask\_c4d');drawnow;
    I(:,:,1) = c_fg(:,:,1);I(:,:,2) = mat2gray(gt)*25;
    figure(114),imshow(uint8(I)),title('gt + real picture');drawnow;
end

%%=================Evaluation===============




