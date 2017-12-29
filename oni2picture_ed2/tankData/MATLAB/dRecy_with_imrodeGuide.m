%% DEPTH_RECOVERY: guided depth recovery with fusioned color mask
function dRecy_with_imrodeGuide(guide, fg_c, fg_d, k, result_file)
    output_addr = '/home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/input/Wajueji_2/dRecvy_use_new_guide/';
    for k = 1:200
%         if nargin<1, mask = imread(['E:\dataSet\Wajueji_2\processedData\182_200\res_',int2str(k),'.png']);end
%         if nargin<2, fg_c = imread(['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png',]);end
%         if nargin<3, fg_d = imread(['E:\dataSet\Wajueji_2\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png']);end
%         if nargin<5, result_file = 'E:\dataSet\Wajueji_2\processedData\182_200\result\';end
        
        
        bg_d = imread('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/depth/fusionedBackgroundData/fusionedBackgroundData.png');
        fg_d = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/depth/fusionedForegroundData/fusionedForegroundData%d.png',...
            k));
        fg_c = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/color/fusionedForegroundData/fusionedForegroundData%d.png',...
            k));
        guide = get_guided(fg_d, bg_d);
        
        mask = imread(sprintf('/home/hhg/Downloads/dataSet/Wajueji_2/processedData/1_200/houqi/d_%d.png',k));
        mask = mask > 0; %binary
        
        %%======depth recovery======
        thres  = 0;
        count = 0;
        recvy_d = zeros(size(mask));
        %%=======compute weight_o======
        [~,weight_o,~] = guided_d_recvy_imerodeGuided(mask, fg_c, -1, count, zeros(1,1), guide, recvy_d);
        count = count +1;
        while 1
            [recvy_d, ~,g_t] = guided_d_recvy_imerodeGuided(mask, fg_c, -1, count, weight_o, guide, recvy_d);
            disp(['processing ',int2str(count), 'th times, now g_t =',int2str(g_t)]);
            if g_t <= thres, break; end
            count = count + 1;
        end
%         D = logical(fg_d) & logical(guide);
%         o2 = sum(sum(D(guide>0) == 0));
%         disp(['frame',int2str(k),': the num of zero in mask before is ',int2str(o1),'   after is ',int2str(o2)]);
%         figure(18), imshow(uint8(mat2gray(D)*255)),title('final result');
%         
        
        I(:,:,1) = (guide>0)*255;
        I(:,:,2) = (recvy_d>0)*255;
        I(:,:,3) = mask*255;
        figure(5),imshow(uint8(I)),title(sprintf('frameNum = %d, guided(R), recvy_d(G), mask(B)',k));drawnow;
        figure(6),imshow(recvy_d,[]),title('recovery depth map');drawnow;
%         res_c(:,:,1) = fg_c(:,:,1) .* uint8(guide);
%         res_c(:,:,2) = fg_c(:,:,2) .* uint8(guide);
%         res_c(:,:,3) = fg_c(:,:,3) .* uint8(guide);
%         imwrite(uint16(res_d), [result_file,'d_',int2str(k),'.png'])
%         imwrite(uint8(res_c), [result_file,'c_',int2str(k),'.png'])
        imwrite(uint16(recvy_d), [output_addr,'d_',int2str(k),'.png']);
        imwrite(uint8(mat2gray(recvy_d)*255), [output_addr,'vis/d_',int2str(k),'.png'])
        disp(['processd ',int2str(k)]);
    end
end


function guide = get_guided(fg_d, bg_d)
    thres = 30;
    mask = abs(bg_d - fg_d) > thres;
    mask = imerode(mask,strel('disk',5));
    guide = double(fg_d) .* double(mask);
end


function [recvy_d, weight_o, g_t]= guided_d_recvy_imerodeGuided(mask, fg_c, ~, count_i, weight, guide, d)
    fg_c = double(fg_c);
    [H, W]  =size(mask);  recvy_d = d;
    win_width = 11;%�������׷���num
    win_height =11; num_win = win_height*win_height;
    half_w = (win_width-1)/2;
    sigma_c = 8;
    thres = 0;
    g_t = 0;%count the num of changing pixels in mask
    sigma_precompute = -2*sigma_c*sigma_c;
    r_start = half_w+1; % round((win_height + 1)/2);  
    r_end = round(H - half_w-1);
    c_start = half_w+1;     c_end = round(W - half_w-1);
    
    G1_mat = fspecial('gaussian', win_width, half_w);
    G1_vec = reshape(mat2gray(G1_mat), 1, num_win);  win_vec = -half_w:half_w;

    weight_o = zeros([H*W,num_win]);
    
    %%only for count weight_o
    if count_i == 0 %
        for r = r_start : r_end
            for c = c_start : c_end          
                vec_patch = reshape(fg_c(r+win_vec, c+win_vec), 1, num_win);
                vec_i = ones(1, num_win)*fg_c(r,c);
                weight_vec = exp((vec_i - vec_patch).^2/sigma_precompute).*G1_vec;%�̶�ֵ�����Ա�������
                weight_o((r-1)*W+c,:) = weight_vec;
            end
        end 
        return;
    end
    
    
    %do all pixels
    for r = r_start : r_end
        for c = c_start : c_end 
            %%only process pixel in mask
            if mask(r,c) > 0 && recvy_d(r,c) == 0
                d_vec = reshape(guide(r+win_vec, c+win_vec), 1, num_win);         
                weight_vec = weight((r-1)*W+c,:);    
                d_m = d_vec >0;
                 s = sum(weight_vec .*d_m);
                 if s == 0   % /s will get NaN
                     res_i = 0;
                 else
                    res_i = sum(weight_vec(d_m).*d_vec(d_m))/s;
                 end
                recvy_d(r,c) = res_i; 
            end
        end
    end
%     zengliang = recvy_d;
%     t = zengliang(zengliang >0);


    Res_I(:,:,1) = recvy_d*255;
    Res_I(:,:,2) = fg_c(:,:,1);
    Res_I(:,:,3) = mask * 255;
    figure(4), imshow(uint8(Res_I)), title('after JBF');drawnow;

    Res_I(:,:,1) = d*255;
    figure(3), imshow(uint8(Res_I)), title('before JBF');drawnow;

%     Res_I(:,:,1) = zengliang*255;
%     figure(16), imshow(uint8(Res_I)), title('zengliang')
%     Res_I(:,:,1) = guide*255;
%     figure(17), imshow(uint8(Res_I)), title('mask')
    g_t = abs(sum(sum(logical(recvy_d) - logical(d))));


end


