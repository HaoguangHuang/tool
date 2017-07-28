function extractedTankData = extractTank_color_Func(fusionedBackgroundData, fusionedForegroundData, extractedDepthMap)
    thres = 10;
    frameNum = size(fusionedForegroundData,2);

    intensityBackground = transfromRGB2Intensity(fusionedBackgroundData);
    for i = 1:frameNum
        intensityForeground(i).data = transfromRGB2Intensity(fusionedForegroundData(i).data);
    end
    
    %%ȡmask
%     matlabpool 4;
    for i = 1:frameNum
        R = fusionedForegroundData(i).data(:,:,1);
        G = fusionedForegroundData(i).data(:,:,2);
        B = fusionedForegroundData(i).data(:,:,3); 
        mask1 = abs((intensityBackground - intensityForeground(i).data)) > thres;
%         mask1 = ones(480,640)
        mask2 = extractedDepthMap(i).data > 0;
        R_new = R .* uint8(mask1) .* uint8(mask2);
        G_new = G .* uint8(mask1) .* uint8(mask2);
        B_new = B .* uint8(mask1) .* uint8(mask2);
        extractedTankData(i).data(:,:,1) = R_new;
        extractedTankData(i).data(:,:,2) = G_new;
        extractedTankData(i).data(:,:,3) = B_new;
    end
%     matlabpool close;
end

    
    
    