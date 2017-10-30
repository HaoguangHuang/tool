function mask_gbf_c = extractTank_color_ycbcr_Func(fu_bg_c, fu_fg_c, k, mask_d4c, mask_c4d, mask_border)
    %%取mask
        global debug_mode;    global gt;
        Y_fg = fu_fg_c(:,:,1);
        Y_bg = fu_bg_c(:,:,1);

        if debug_mode, figure(888),imshow(mask_c4d,[]),title('color intensity substract');drawnow; end;

        t = tic;
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        if 1
%         [~, weight_o,~]= guided_JBF(mask_c4d, Y_fg,-1,count, weight_i,3,5,0.5);%这里只是计算了weight_o
        [~, weight_o,~,weight_s]= guided_JBF_opt(mask_c4d, Y_fg,-1,count, weight_i,6,7,0.5);%这里只是计算了weight_o
        count = count + 1;
        g_thres = 10;%guided thres------per pixel
        g_t = inf;
        while count<5
%             [mask_c4d, ~, g_t] = guided_JBF(mask_c4d, Y_fg, -1, count, weight_o,3,5,0.5);%guided imdilate
            [mask_c4d, ~, g_t,~] = guided_JBF_opt(mask_c4d, Y_fg, -1, count, weight_o,6,7,0.5,weight_s);
            if g_t <= g_thres
                disp(['frame ',int2str(k), '------------total for ', int2str(count), ' times!']);
                break;
            end
            count = count + 1;
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th time']);
        end
        end
        mask_gbf_c = logical(mask_c4d);
%         I(:,:,1) = mask_gbf_c*255;
%         I(:,:,2) = Y_fg;
%         I(:,:,3) = mat2gray(gt)*255;
%         if debug_mode, figure(666),imshow(uint8(I));end;
        mask_gbf_c = logical(mask_d4c) | logical(mask_gbf_c) & logical(mask_border);
        disp(['color JBF=',num2str(toc(t))]);
        I(:,:,1) = mask_gbf_c*255;
%         if debug_mode, figure(666),imshow(uint8(I));drawnow; end;
    end