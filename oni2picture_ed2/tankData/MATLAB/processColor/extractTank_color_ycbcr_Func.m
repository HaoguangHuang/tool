function mask_gbf_c = extractTank_color_ycbcr_Func(fusionedBackgroundData, fusionedForegroundData, k,...
    mask_d4c, mask_c4d, series)
    %%取mask
        global debug_mode;    global gt;
        Y_fg = fusionedForegroundData(:,:,1);
        Y_bg = fusionedBackgroundData(:,:,1);

        if debug_mode, figure(888),imshow(mask_c4d,[]),title('color intensity substract');drawnow; end;
        
        mask_c4d = logical(mask_d4c) .* mask_c4d;
        if debug_mode, figure(666),imshow(mask_c4d,[]),title('mask1\_d .* mask1');drawnow; end;

        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(mask_c4d, Y_fg,1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;
        g_thres = 0;%guided thres------per pixel
        g_t = inf;
        while 1
            [mask_c4d, ~, g_t] = guided_JBF(mask_c4d, Y_fg, -1, count, weight_o);%guided imdilate
            if g_t <= g_thres
                disp(['frame ',int2str(k), '------------total for ', int2str(count), ' times!']);
                break;
            end
            count = count + 1;
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th time']);
        end
        
        mask_gbf_c = mask_c4d;
        I(:,:,1) = mask_gbf_c*255;
        I(:,:,2) = Y_fg;
        I(:,:,3) = mat2gray(gt)*255;
        if debug_mode, figure(666),imshow(uint8(I));end;
        mask_gbf_c = logical(mask_d4c) + logical(mask_gbf_c);
        I(:,:,1) = mask_gbf_c*255;
        if debug_mode, figure(666),imshow(uint8(I));drawnow; end;
%         imwrite(uint8(mask_gbf_c),['E:\dataSet\Wajueji_2\processedData\c_Mask_4d\mask',int2str(k),'_c.png']);
    end