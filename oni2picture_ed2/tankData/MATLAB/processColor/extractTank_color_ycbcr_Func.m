function extractedTankData = extractTank_color_ycbcr_Func(fusionedBackgroundData, fusionedForegroundData, extractedDepthMap)
    thres = 60;
    frameNum = 1;%size(fusionedForegroundData,2);
    
    %%取mask
    for i = 1:frameNum
        Y_fg = fusionedForegroundData(:,:,1);
        Y_bg = fusionedBackgroundData(:,:,1);

        mask1 = abs(double(Y_bg) - double(Y_fg)) > thres;
%         mask1 = ones(480,640);
    %%对mask1膨胀腐蚀
%         se = strel('rectangle', [5 5]);
%         mask1 = imdilate(mask1, se);
%         mask1 = imerode(mask1, se);      
       
        mask2 = extractedDepthMap(i).data > 0;
%          mask2 = ones(480,640);

%         figure, imshow(Y_fg,[]);
%         figure, imshow(Y_bg,[]);
        figure, imshow(mask1,[]),title('frame143, mask1 from ycbcr');  
         figure, imshow(mask2,[]), title('frame143, mask2 from extractedDepth');

        extractedTankData(i).data = zeros(size(fusionedBackgroundData));
        index = mask1 .* mask2;%mask与操作
%         index = (mask1 + mask2) > 0;%mask或操作
        extractedTankData(i).data(:,:,1) = uint8(fusionedForegroundData(:,:,1)) .* uint8(index);
        extractedTankData(i).data(:,:,2) = uint8(fusionedForegroundData(:,:,2)) .* uint8(index);
        extractedTankData(i).data(:,:,3) = uint8(fusionedForegroundData(:,:,3)) .* uint8(index);

    end
end