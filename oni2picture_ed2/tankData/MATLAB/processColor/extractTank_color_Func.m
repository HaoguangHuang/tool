%% processing rgb
function extractedTankData = extractTank_color_Func(fusionedBackgroundData, fusionedForegroundData, extractedDepthMap)
    thres = 10;
    frameNum = size(fusionedForegroundData,2);

     intensityBackground = transfromRGB2Intensity(fusionedBackgroundData);

    for i = 1:frameNum
        intensityForeground(i).data = transfromRGB2Intensity(fusionedForegroundData(i).data);
    end
    
    %%È¡mask
    for i = 1:frameNum
        R = fusionedForegroundData(i).data(:,:,1);
        G = fusionedForegroundData(i).data(:,:,2);
        B = fusionedForegroundData(i).data(:,:,3); 
        mask1 = abs((double(intensityBackground) - double(intensityForeground(i).data))) > thres;
%         mask1 = ones(480,640);
    %%¶Ômask1ÅòÕÍ¸¯Ê´
%         se = strel('rectangle', [5 5]);
%         mask1 = imdilate(mask1, se);
%         mask1 = imerode(mask1, se);      
       
        mask2 = extractedDepthMap(i).data > 0;
%          mask2 = ones(480,640);
        R_new = R .* uint8(mask1) .* uint8(mask2);
        G_new = G .* uint8(mask1) .* uint8(mask2);
        B_new = B .* uint8(mask1) .* uint8(mask2);
        extractedTankData(i).data(:,:,1) = R_new;
        extractedTankData(i).data(:,:,2) = G_new;
        extractedTankData(i).data(:,:,3) = B_new;
    end
end

    
    
    