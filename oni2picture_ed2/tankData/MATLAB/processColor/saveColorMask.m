function m = saveColorMask(fu_bg_c, fu_fg_c, k, subs_file, series)
    thres = 10;
    global debug_mode;
    Y_fg = fu_fg_c(:,:,1);
    Y_bg = fu_bg_c(:,:,1);
    gt = imread(['E:\dataSet\ICAISS\Hybrid_FBS\',series,'\',series,'\','groundTruth\gt_',int2str(k),'.bmp']);
    mask1 = abs(double(Y_bg) - double(Y_fg)) > thres;
%     imwrite(uint8(mask1), [subs_file,'mask',int2str(k),'_c.png']);
    I(:,:,1) = mat2gray(mask1)*255;
    I(:,:,2) = Y_fg;
    I(:,:,3) = mat2gray(gt)*255;
    if debug_mode, figure(140),imshow(uint8(I)),title('color fg/bg substraction');drawnow; end;
    m = mask1;
end