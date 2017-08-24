function m = saveColorMask(fusionedBackgroundData, fusionedForegroundData, k, subs_file)
    thres = 30;
    Y_fg = fusionedForegroundData(:,:,1);
    Y_bg = fusionedBackgroundData(:,:,1);
    
    mask1 = abs(double(Y_bg) - double(Y_fg)) > thres;
%     imwrite(uint8(mask1), ['E:\dataSet\Wajueji_2\processedData\intensityMask\mask',int2str(k),'_c.png']);
    imwrite(uint8(mask1), [subs_file,'mask',int2str(k),'_c.png']);
    m = mask1;
end