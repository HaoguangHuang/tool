function extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData,ycbcr_mat, k)
    fusionedForeFraNum = size(fusionedForegroundData, 2);
    thres = 15;%mm
    result = {};
    %%前景减背景
    
    for i = 1:fusionedForeFraNum
        mask1 =  (double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data)))>thres;
        mask2 = fusionedForegroundData(i).data < 1000;
        mask1_c = imread(['E:\dataSet\Wajueji_2\processedData\c_Mask_4d\mask',int2str(k),'_c.png']);
        mask_1_2 = mask1 .* mask2;
        I(:,:,1) = mask_1_2*255;
        I(:,:,2) = ycbcr_mat(:,:,1);
        I(:,:,3) = zeros(size(fusionedBackgroundData));
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        
        mask_1_2 = imerode(mask_1_2, strel('disk', 5));
        I(:,:,1) = mask_1_2*255;
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        
        
        mask_1_2 = logical(mask_1_2) + logical(mask1_c);
         I(:,:,1) = mask_1_2*255;
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        mask_1_2 = mask_1_2 .* mask2;
         I(:,:,1) = mask_1_2*255;
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        %%
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(mask_1_2, ycbcr_mat(:,:,1),1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;
        g_thres = 50;%guided thres------per pixel
        g_t = 100000000;
        while 1
            [mask_1_2, ~, g_t] = guided_JBF(mask_1_2, ycbcr_mat(:,:,1), -1, count, weight_o);%guided imdilate
            if g_t < g_thres
                disp(['frame ',int2str(k), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;
            
        end
        mask_gbf_d = mask_1_2;
        imwrite(uint8(mask_gbf_d), ['E:\dataSet\Wajueji_2\processedData\mask_d_c\mask',int2str(k),'_d_c.png']);
        I(:,:,1) = mat2gray(mask_gbf_d)*255;
        I(:,:,2) = fusionedForegroundData(i).data;
        I(:,:,3) = zeros(size(mask_gbf_d));
        imwrite(uint8(I), ['E:\dataSet\Wajueji_2\processedData\mask_d_c\vis_mask',int2str(k),'_d_c.png'])
        result(i).data = uint16(fusionedForegroundData(i).data) .* uint16(mask_gbf_d);

%%转化到[0-255]
%          result(i).data = double(result(i).data) .* (255/10000);       
  
    end
 
    extractedTankData = result;
end