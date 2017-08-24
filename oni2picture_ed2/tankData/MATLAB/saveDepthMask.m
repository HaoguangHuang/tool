function m = saveDepthMask(fusionedBackgroundData, fusionedForegroundData, k, subs_file, fu_fg_c)
    fb_thres = 15;
    fore_thres = 1000;
    mask1_d =  (double(fusionedBackgroundData) - (double(fusionedForegroundData)))>fb_thres;
    mask2_d = (fusionedForegroundData < fore_thres) .* (fusionedForegroundData > 0) ;
    mask1_d = mask1_d .* mask2_d;
    I(:,:,1) = mat2gray(mask1_d)*255;
    I(:,:,2) = fu_fg_c(:,:,1);
    I(:,:,3) = zeros(size(fu_fg_c(:,:,1)));
    figure(888),imshow(uint8(I));
    
%     figure(888),imshow(mat2gray(mask1_d));
%     mask1_d = imerode(mask1_d, strel('disk', 3));
%     
%     I(:,:,1) = mat2gray(mask1_d)*255;
%     figure(666),imshow(uint8(I));
%     figure(666),imshow(mat2gray(mask1_d));
    imwrite(uint8(mask1_d), [subs_file,'mask',int2str(k),'_d.png']);
    m = mask1_d;  
    
end