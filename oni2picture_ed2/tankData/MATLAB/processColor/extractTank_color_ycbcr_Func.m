function mask_gbf_c = extractTank_color_ycbcr_Func(fusionedBackgroundData, fusionedForegroundData, k,...
    mask1_d4c, mask1)
    %%取mask
        Y_fg = fusionedForegroundData(:,:,1);
        Y_bg = fusionedBackgroundData(:,:,1);

        figure(888),imshow(mask1,[]),title('color intensity substract');
        
        mask1 = logical(mask1_d4c) .* mask1;
        figure(666),imshow(mask1,[]),title('mask1\_d .* mask1'); 

        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(mask1, Y_fg,1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;
        g_thres = 10;%guided thres------per pixel
        g_t = 100000000;
        while 1
            [mask1, ~, g_t] = guided_JBF(mask1, Y_fg, -1, count, weight_o);%guided imdilate
            if g_t < g_thres
                disp(['frame ',int2str(k), '------------total for ', int2str(count), ' times!']);
                break;
            end
            count = count + 1;
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th time']);
        end
        mask_gbf_c = mask1;
   
        mask_gbf_c = logical(mask1_d4c) + logical(mask_gbf_c);
        I(:,:,1) = mask_gbf_c*255;
        I(:,:,2) = Y_fg;
        I(:,:,3) = zeros(size(mask_gbf_c));
        figure(666),imshow(uint8(I));
        imwrite(uint8(mask_gbf_c),['E:\dataSet\Wajueji_2\processedData\c_Mask_4d\mask',int2str(k),'_c.png']);
    end