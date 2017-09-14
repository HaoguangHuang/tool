%% TEST_MAIN: this script support to test fg/bg segmentation in dataset
%       ShSeq:420-540     

% clear all; close all;
warning('off');   
global debug_mode;  debug_mode = 0;  
series = 'GenSeq'; root_dir = ['E:\dataSet\ICAISS\Hybrid_FBS\' series '\'];
frameStart = 924; frameEnd = 1038;
%%==================background fusion==================
if 0
[c_bg, d_bg] = Extract_BG_From_DataSet([root_dir series]);
GenSeq_d_bg = d_bg;
GenSeq_c_bg = c_bg;
save('data.mat',[series,'_d_bg'],'-append');
save('data.mat',[series,'_c_bg'],'-append');
else
    load('data.mat',[series,'_d_bg'],[series,'_c_bg']);
    d_bg = GenSeq_d_bg;  c_bg = GenSeq_c_bg;
end
%%=================foreground substraction===============
for k = 981:3:frameEnd
    if k == 939, continue; end
if 1
    d_fg = imread([root_dir '\' series '\depthData\depth_' int2str(k) '.png']);
    c_fg = imread([root_dir '\' series '\ycbcrData\ycbcr_color_' int2str(k) '.png']);
    output_file = [root_dir '\' series '\output\'];
    mask_d4c = saveDepthMask(d_bg, d_fg, k, [], c_fg, series);
    mask_c4d = saveColorMask(c_bg, c_fg, k, [], series);
    %%======process ycbcr map======%%
    if 1
        mask_gbf_c = extractTank_color_ycbcr_Func(c_bg, c_fg, k, mask_d4c, mask_c4d, series);
    end
else
    load('mask_gbf_c.mat',[series, '_mask_gbf_c']); mask_gbf_c = GenSeq_mask_gbf_c;
end

if 1
    %%======process depth map======%%
    mask = extractTankFunc(d_bg, d_fg, c_fg, k, mask_gbf_c, output_file, mask_d4c, series);
    %%======compare with gt=======%%
    if debug_mode
    gt = imread([root_dir '\' series '\groundTruth\gt_' int2str(k) '.bmp']);
    I(:,:,1) = mat2gray(mask)*255; I(:,:,2) = mat2gray(gt)*255; I(:,:,3) = zeros(size(gt));
    figure(111),imshow(uint8(I));title('gt + mask\_result'),drawnow;
    I(:,:,1) = mat2gray(mask_d4c)*255;
    figure(112),imshow(uint8(I));title('gt + mask\_d4c');drawnow;
    I(:,:,1) = mat2gray(mask_c4d)*255;
    figure(113),imshow(uint8(I)),title('gt + mask\_c4d');drawnow;
    I(:,:,1) = c_fg(:,:,1);I(:,:,2) = mat2gray(gt)*25;
    figure(114),imshow(uint8(I)),title('gt + real picture');drawnow;
    end
end
end

%%=================Evaluation===============
if 0
[mean_res, res_mat] = Evaluation_RGBD_new(['E:\dataSet\ICAISS\Hybrid_FBS\',series,'\',series],...
                                        'backUp',...
                                        'png',...
                                        'groundTruth');
end


