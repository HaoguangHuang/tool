%% AFTER_PROC: PROCESS RESULTS IN ./OUTPUT, USING GUIDED_JBF
%         Note: this process is suit to ShSeq series
function after_Proc(k, root_dir)
    if nargin < 1, k = [450,470]; end;
    if nargin < 2, root_dir = 'E:\dataSet\ICAISS\Hybrid_FBS\ShSeq\ShSeq\';end
    global debug_mode; debug_mode = 1;
    bg_d = load('data.mat','d_bg'); bg_d = bg_d.d_bg;
    for i = 455
        fg_d = imread([root_dir '\depthData\depth_' int2str(i) '.png']);
        mask = imread([root_dir 'output\res_' int2str(i) '.png']);
        fg_c = imread([root_dir 'ycbcrData\ycbcr_color_' int2str(i) '.png']);
        mask_d4c = saveDepthMask(bg_d, fg_d, i, [], fg_c);
        gt = imread([root_dir 'groundTruth\gt_' int2str(i) 'BW.bmp']);
        I(:,:,1) = mat2gray(mask)*255; I(:,:,2) = fg_c(:,:,1); I(:,:,3) = mat2gray(gt)*255;
        if debug_mode, figure(60),imshow(uint8(I)),title('raw mask');drawnow; end
        %%=======================================================
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(double(mask), fg_c(:,:,1),1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;  g_thres = 0;%guided thres------per pixel     
        g_t = inf;
        while 1
            [mask, ~, g_t] = guided_JBF(double(mask), fg_c(:,:,1), -1, count, weight_o);%guided imdilate
            if g_t <= g_thres
                disp(['frame ',int2str(i), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;
        end
        I(:,:,1) = mat2gray(mask)*255;
        if debug_mode, figure(61),imshow(uint8(I)),title('mask after JBF'); end;  
        mask = (mask + mask_d4c)>0;
        I(:,:,1) = mat2gray(mask)*255;
        if debug_mode, figure(61),imshow(uint8(I)),title(['frame ', int2str(i), '----res + mask\_d4c']);end;
        %%=======================================================
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(double(mask), fg_c(:,:,1),1,count, weight_i);%这里只是计算了weight_o
        count = count + 1;  g_thres = 0;%guided thres------per pixel     
        g_t = inf;
        while 1
            [mask, ~, g_t] = guided_JBF(double(mask), fg_c(:,:,1), -1, count, weight_o);%guided imdilate
            if g_t <= g_thres
                disp(['frame ',int2str(i), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;
        end
        I(:,:,1) = mat2gray(mask)*255;
        if debug_mode,figure(62),imshow(uint8(I)),title(['frame ', int2str(i), '----res + mask\_d4c+JBF']);end
        imwrite(uint8(I), [root_dir 'bestRes\vis\res_' int2str(i) '.png']);
        imwrite(mask, [root_dir 'bestRes\res_' int2str(i) '.png']);
    end
end