%% DEPTH_RECOVERY: guided depth recovery with fusioned color mask
function depthRecovery(mask, fg_c, fg_d, k, result_file)
%         m_name = ['E:\dataSet\Wajueji_2\processedData\mask_d_c\mask',int2str(k),'_d_c_gd50gc50.png'];
%         fg_c_name = ['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png',]; 
%         fg_d_name = ['E:\dataSet\Wajueji_2\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png'];
%         mask = imread(m_name); fg_c = imread(fg_c_name); fg_d = imread(fg_d_name);

%     for k = 183:200
        if nargin<1, mask = imread(['E:\dataSet\Wajueji_2\processedData\182_200\res_',int2str(k),'.png']);end
        if nargin<2, fg_c = imread(['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png',]);end
        if nargin<3, fg_d = imread(['E:\dataSet\Wajueji_2\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png']);end
        if nargin<5, result_file = 'E:\dataSet\Wajueji_2\processedData\182_200\result\';end
        %%======depth recovery======
        thres  = 0;
        count = 0;
        
        fg_d = double(fg_d) .* double(mask);
        %%=======compute weight_o======
        [~,weight_o,~] = guided_d_recvy(fg_d, fg_c, -1, count, zeros(1,1), mask);
        count = count +1;
        D = fg_d(mask>0);
        o1 = sum(sum(D == 0));
        while 1
            [fg_d, ~,g_t] = guided_d_recvy(fg_d, fg_c, -1, count, weight_o, mask);
            disp(['processing ',int2str(count), 'th times, now g_t =',int2str(g_t)]);
            if g_t <= thres, break; end
            count = count + 1;
        end
        D = logical(fg_d) & logical(mask);
        o2 = sum(sum(D(mask>0) == 0));
        disp(['frame',int2str(k),': the num of zero in mask before is ',int2str(o1),'   after is ',int2str(o2)]);
        figure(18), imshow(uint8(mat2gray(D)*255)),title('final result');
        
        res_d = double(fg_d) .* double(mask);
        res_c(:,:,1) = fg_c(:,:,1) .* uint8(mask);
        res_c(:,:,2) = fg_c(:,:,2) .* uint8(mask);
        res_c(:,:,3) = fg_c(:,:,3) .* uint8(mask);
        imwrite(uint16(res_d), [result_file,'d_',int2str(k),'.png'])
        imwrite(uint8(res_c), [result_file,'c_',int2str(k),'.png'])
%     end
end