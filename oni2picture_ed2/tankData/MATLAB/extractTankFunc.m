function mask = extractTankFunc(fusionedBackgroundData, fusionedForegroundData,fu_fg_c, k, mask_gbf_c...
    , mask_file, mask_1_2)
        mask2 = (fusionedForegroundData < 1000) .* (fusionedForegroundData > 0) ;
        %%%%%
        I(:,:,1) = mat2gray(mask_1_2)*255;
        I(:,:,2) = fu_fg_c(:,:,1);
        I(:,:,3) = zeros(size(fusionedBackgroundData));
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');

        mask_1_2 = logical(mask_1_2) + logical(mask_gbf_c);
         I(:,:,1) = mask_1_2*255;
        figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
%         mask_1_2 = mask_1_2 .* mask2;
%          I(:,:,1) = mask_1_2*255;
%         figure(10),imshow(uint8(I),[]),title('maks\_1\_2');
        %%
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(mask_1_2, fu_fg_c(:,:,1),1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;
        g_thres = 10;%guided thres------per pixel
        g_t = 100000000;
        while 1
            [mask_1_2, ~, g_t] = guided_JBF(mask_1_2, fu_fg_c(:,:,1), -1, count, weight_o);%guided imdilate
            if g_t < g_thres
                disp(['frame ',int2str(k), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;
            
        end
        mask_gbf_d = mask_1_2;
        imwrite(uint8(mask_gbf_d), [mask_file,'mask',int2str(k),'_d_c_gd50gc50.png']);
        I(:,:,1) = mask_gbf_d*255;
        I(:,:,2) = fu_fg_c(:,:,1);
        I(:,:,3) = zeros(size(mask_gbf_d));
        figure(18),imshow(uint8(I)),title('final result');
        imwrite(uint8(I), [mask_file,'\vis\vis_mask',int2str(k),'_d_c_gd50gc50.png'])
%         result(1).data = uint16(fusionedForegroundData) .* uint16(mask_gbf_d);
        
        mask = mask_gbf_d;
end