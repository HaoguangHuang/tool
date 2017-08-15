function saveDepthMask(fusionedBackgroundData, fusionedForegroundData, k)
    thres = 60;
    mask1_d =  (double(fusionedBackgroundData) - (double(fusionedForegroundData(1).data)))>thres;
    mask2_d = fusionedForegroundData(1).data < 1000;
    mask1_d = mask1_d .* mask2_d;
    imwrite(uint8(mask1_d), ['C:\Users\hhg\Desktop\tmp\mask',int2str(k),'_d.png']);
end