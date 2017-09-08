%% SAVE_DEPTH_MASK:depth foreground substraction
function m = saveDepthMask(fu_bg_d, fu_fg_d, k, ~, fu_fg_c)
    fb_thres = 15;
    fore_thres = 1800;
    gt = imread(['E:\dataSet\ICAISS\Hybrid_FBS\ShSeq\ShSeq\groundTruth\gt_',int2str(k),'BW.bmp']);
    mask1_d =  abs(double(fu_bg_d) - (double(fu_fg_d)))>fb_thres;
    mask2_d = (fu_fg_d < fore_thres) .* (fu_fg_d >= 0) ;
    mask1_d = mask1_d .* mask2_d;
    mask1_d = imerode(mask1_d,strel('disk',3));
    mask1_d = imerode(mask1_d,strel('disk',5));
    I(:,:,1) = mat2gray(mask1_d)*255;
    I(:,:,2) = fu_fg_c(:,:,1);
    I(:,:,3) = mat2gray(gt)*255;
    figure(888),imshow(uint8(I));drawnow;

    m = mask1_d;  
end