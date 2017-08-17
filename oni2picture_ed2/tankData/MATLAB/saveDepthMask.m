function saveDepthMask(fusionedBackgroundData, fusionedForegroundData, k)
    thres = 15;
    mask1_d =  (double(fusionedBackgroundData) - (double(fusionedForegroundData(1).data)))>thres;
    mask2_d = fusionedForegroundData(1).data < 1000;
    mask1_d = mask1_d .* mask2_d;
    mask1_d = imerode(mask1_d, strel('disk', 5));
    imwrite(uint8(mask1_d), ['E:\dataSet\Wajueji_2\processedData\d_Mask_4c\mask',int2str(k),'_d.png']);
end