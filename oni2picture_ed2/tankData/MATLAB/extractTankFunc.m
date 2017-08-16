function extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData,ycbcr_mat, k)
    fusionedForeFraNum = size(fusionedForegroundData, 2);
    thres = 60;%mm
    result = {};
    %%前景减背景
    
    for i = 1:fusionedForeFraNum
        mask1 =  (double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data)))>thres;
        mask2 = fusionedForegroundData(i).data < 1000;
        mask1_c = imread(['E:\dataSet\Wajueji_2\processedData\intensityMask\mask',int2str(k),'_c.png']);
        mask_1_2 = mask1 .* mask2;
        I(:,:,1) = mask_1_2*255;
        I(:,:,2) = ycbcr_mat(:,:,1);
        I(:,:,3) = zeros(size(mask_1_2));
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        
        mask_1_2 = imerode(mask_1_2, strel('disk', 5));
        I(:,:,1) = mask_1_2*255;
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        
        mask_1_2 = ((mask_1_2 + double(mask1_c))>0) .* mask2;
        I(:,:,1) = mask_1_2*255;
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        
        
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, ~, weight_o]= guided_JBF(mask_1_2, ycbcr_mat(:,:,1),1,count, weight_i);%这里只是计算了weight_o
        
        [mask_a, ~, ~]= guided_JBF(mask_1_2, ycbcr_mat(:,:,1),1,count, weight_o);
        [mask_a, ~, ~] = guided_JBF(mask_a, ycbcr_mat(:,:,1),0,count, weight_o);
        [mask_a, ~, ~] = guided_JBF(mask_a, ycbcr_mat(:,:,1),0,count, weight_o);
        [mask_a, ~, ~] = guided_JBF(mask_a, ycbcr_mat(:,:,1),0,count, weight_o);
        [mask_a, ~, ~] = guided_JBF(mask_a, ycbcr_mat(:,:,1),0,count, weight_o);
        [mask_gbf_d, ~, ~] = guided_JBF(mask_a, ycbcr_mat(:,:,1),0,count, weight_o);
       
        
        
        I(:,:,1) = mask_gbf_d*255;
        figure(111), imshow(uint8(I),[]), title('depth mask\_jbf\_d = processed(mask\_1\_2)');
         I(:,:,1) = mask_1_2*255;
        figure(222), imshow(uint8(I), []), title('depth mask\_1\_2');
        
        result(i).data = uint16(fusionedForegroundData(i).data) .* uint16(mask_gbf_d);

%%转化到[0-255]
%          result(i).data = double(result(i).data) .* (255/10000);       
    end   
    extractedTankData = result;
end