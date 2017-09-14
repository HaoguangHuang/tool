function mask = extractTankFunc(fusionedBackgroundData, fusionedForegroundData,fu_fg_c, k, mask_gbf_c...
    , mask_file, mask_d4c, series)
        global debug_mode;
        mask2 = (fusionedForegroundData < 2100) .* (fusionedForegroundData > 0) ;
        %%%%%
        gt = imread(['E:\dataSet\ICAISS\Hybrid_FBS\',series,'\',series,'\','groundTruth\gt_',int2str(k),'.bmp']);
        I(:,:,1) = mat2gray(mask_d4c)*255;
        I(:,:,2) = fu_fg_c(:,:,1);
        I(:,:,3) = mat2gray(gt)*255;
        if debug_mode ,figure(10),imshow(uint8(I),[]),title('maks\_d4c');drawnow; end;

        mask_d4c = logical(mask_d4c) + logical(mask_gbf_c);
         I(:,:,1) = mask_d4c*255;
        if debug_mode, figure(10),imshow(uint8(I),[]),title('maks\_d4c+mask_gbf_c');drawnow; end;

        %%
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(mask_d4c, fu_fg_c(:,:,1),1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;
        g_thres = 0;%guided thres------per pixel
        g_t = 100000000;
        while 1
            [mask_d4c, ~, g_t] = guided_JBF(mask_d4c, fu_fg_c(:,:,1), -1, count, weight_o);%guided imdilate
            if g_t <= g_thres
                disp(['frame ',int2str(k), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;
            
        end
        mask_gbf_d = mask_d4c;
        imwrite(uint8(mask_gbf_d), [mask_file,'res_',int2str(k),'.png']);
        mask_gbf_d = mask_gbf_d + mask_d4c;
        
        I(:,:,1) = mask_gbf_d*255;
        I(:,:,2) = fu_fg_c(:,:,1);
        I(:,:,3) = mat2gray(gt)*255;
        if debug_mode, figure(18),imshow(uint8(I)),title('final result');drawnow; end;
        imwrite(uint8(I), [mask_file,'vis\res_',int2str(k),'.png'])
%         result(1).data = uint16(fusionedForegroundData) .* uint16(mask_gbf_d);
        
        mask = mask_gbf_d;
end