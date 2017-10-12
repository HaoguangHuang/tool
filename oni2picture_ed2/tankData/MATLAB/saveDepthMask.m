%% SAVE_DEPTH_MASK:depth foreground substraction
function m = saveDepthMask(fu_bg_d, fu_fg_d, ~, ~, fu_fg_c)
    fb_thres = 40;
    fore_thres = 1000;
    global debug_mode;   global gt;
    
    I(:,:,2) = fu_fg_c(:,:,1);
    I(:,:,3) = mat2gray(gt)*255;
    
    mask1_d =  abs(double(fu_bg_d) - (double(fu_fg_d)))>fb_thres;
    I(:,:,1) = mat2gray(mask1_d)*255;
    if debug_mode, figure(888),imshow(uint8(I));drawnow; end;
    
    mask2_d = fu_fg_d < fore_thres & fu_fg_d > 0;
    I(:,:,1) = mat2gray(mask2_d)*255;
    if debug_mode, figure(888),imshow(uint8(I));drawnow; end;
    
    mask1_d = logical(mask1_d) & mask2_d;
    I(:,:,1) = mat2gray(mask1_d)*255;
    if debug_mode, figure(888),imshow(uint8(I));drawnow; end;
    
%     mask1_d = imerode(mask1_d,strel('disk',5));
%     I(:,:,1) = mat2gray(mask1_d)*255;
%     if debug_mode, figure(888),imshow(uint8(I));drawnow; end;
%     
%     mask1_d = imerode(mask1_d,strel('disk',5));
%     I(:,:,1) = mat2gray(mask1_d)*255;
%     if debug_mode, figure(888),imshow(uint8(I));drawnow; end;
    
    mask1_d = imerode(mask1_d,strel('disk',6));
    I(:,:,1) = mat2gray(mask1_d)*255;
    if debug_mode, figure(888),imshow(uint8(I));drawnow; end;
    
    if debug_mode, figure(1),imshow(fu_fg_d,[]);figure(2),imshow(fu_bg_d,[]); end;
    m = mask1_d;  
end