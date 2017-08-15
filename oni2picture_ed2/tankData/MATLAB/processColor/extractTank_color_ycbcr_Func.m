function extractedTankData_c = extractTank_color_ycbcr_Func(fusionedBackgroundData, fusionedForegroundData, extractedDepthMap, k)
    thres =30;
    frameNum = 1;%size(fusionedForegroundData,2);
    mask1_d = imread(['E:\dataSet\Wajueji_2\processedData\intensityMask\mask',int2str(k),'_d.png']);
    %%取mask
    for i = 1:frameNum
        Y_fg = fusionedForegroundData(:,:,1);
        Y_bg = fusionedBackgroundData(:,:,1);

        mask1 = abs(double(Y_bg) - double(Y_fg)) > thres;
        figure(888),imshow(mask1,[]),title(['thres = ',int2str(thres),',  color intensity substract']);
        figure(123213),imshow(mask1_d,[]);
        mask1 = (mask1_d>0) .* mask1;
        figure(666),imshow(mask1,[]),title('mask1\_d .* mask1');
        
        mask_jbf_d = extractedDepthMap(i).data > 0; 
        mask1 = (mask1 + mask_jbf_d)>0;
        figure(999),imshow(mask1,[]),title('mask1 + mask\_jbf\_d');
    %%对mask1膨胀腐蚀
        se1 = strel('rectangle', [3 3]);
        se2 = strel('rectangle', [5 5]);
        figure(6), imshow(uint8(mask1),[]),title('color before imerode imdilate ');
        mask1 = imdilate(mask1, se2);
%         mask1 = imdilate(mask1, se2);
%         mask1 = imdilate(mask1, se2);
        figure(2), imshow(uint8(mask1),[]),title('color after imerode imdilate ');
    %%guided膨胀
        weight_i = zeros(1,1);
        count = 0;%如果做膨胀后又做腐蚀，那count需要从0开始
        [mask_1, ~,weight_o] = guided_JBF(mask1, Y_fg, 0,count, weight_i);%
        [mask_1, ~,~] = guided_JBF(mask1, Y_fg, 0,count, weight_o);
        [mask_1, ~,~] = guided_JBF(mask1, Y_fg, 0,count, weight_o);
        [mask_1, ~,~] = guided_JBF(mask1, Y_fg, 0,count, weight_o);
        [mask_jbf_c, ~,~] = guided_JBF(mask1, Y_fg, 0,count, weight_o);
        
            
        figure(5),imshow(extractedDepthMap(i).data,[]),title('mask_jbf_d');
        extractedTankData_c(i).data = zeros(size(fusionedBackgroundData));
%         index = mask1 .* mask2;%mask与操作
%             index = mask1_jbf .* mask2;
             index1 = (mask_jbf_d .* mask_jbf_c)>0;
             index2 = (mask_jbf_d + mask_jbf_c)>0;
%         index = (mask1 + mask2) > 0;%mask或操作
        I(:,:,1) = index1*255;
        I(:,:,2) = Y_fg;
        I(:,:,3) = zeros(size(index1));
        figure(7),imshow(uint8(I)),title('mask\_jbf\_d .* mask\_jbf\_c');
        I(:,:,1) = index2*255;
        figure(8),imshow(uint8(I)),title('mask\_jbf\_d + mask\_jbf\_c');
        extractedTankData_c(i).data(:,:,1) = uint8(fusionedForegroundData(:,:,1)) .* uint8(index1);
        extractedTankData_c(i).data(:,:,2) = uint8(fusionedForegroundData(:,:,2)) .* uint8(index1);
        extractedTankData_c(i).data(:,:,3) = uint8(fusionedForegroundData(:,:,3)) .* uint8(index1);
        imwrite(uint8(index1),'E:\dataSet\Wajueji_2\processedData\finalMask\mask',int2str(k),'.png');
    end
end