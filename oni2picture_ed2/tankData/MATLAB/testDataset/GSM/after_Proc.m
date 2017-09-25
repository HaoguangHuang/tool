%% AFTER_PROC: PROCESS RESULTS IN ./OUTPUT, USING GUIDED_JBF
%         Note: this process is suit to ShSeq series
function after_Proc(k, root_dir)
    if nargin < 1, k = [450,470]; end;
    if nargin < 2, root_dir = 'E:\dataSet\ICAISS\GSM\GSM_dataset\GSM\Sleeping_ds\';end
    global debug_mode; debug_mode = 1; 
    global gt;
    series = 'Sleeping_ds';
    load('background.mat',[series,'_d_bg_250_299']); eval(['bg_d = ',series, '_d_bg_250_299']);
    load('background.mat',[series,'_c_bg_250_299']); eval(['bg_c = ',series, '_c_bg_250_299']);
    for i = 225
        mask = imread([root_dir 'output\res_' int2str(i) '.png']);
        fg_c = imread([root_dir 'ycbcrData\ycbcr_color_' int2str(i) '.png']);
        fg_d = imread([root_dir 'depthData\depth_' int2str(i) '.png']);
        gt = imread([root_dir 'groundTruth\' int2str(i) '.bmp']);
        mask_c4d = saveColorMask(bg_c, fg_c);
         mask_d4c = saveDepthMask(bg_d, fg_d, i, '', fg_c);
         mask_bg = fg_d < 2000;
        I(:,:,1) = mat2gray(mask)*255; I(:,:,2) = fg_c(:,:,1); I(:,:,3) = mat2gray(gt)*255;
        if debug_mode, figure(60),imshow(uint8(I)),title('raw mask');drawnow; end
        
        mask = logical(mask) | logical(mask_c4d) | mask_d4c;
        I(:,:,1) = mat2gray(mask)*255;
        if debug_mode, figure(60),imshow(uint8(I)),title('raw mask + mask\_c4d');drawnow; end
        
        %%=======================================================
                                                                                                                                 count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(double(mask), fg_c(:,:,1),1,count, weight_i,3,5,0.3);%这里只是计算了weight_o
        count = count + 1;  g_thres = 0;%guided thres------per pixel     
        g_t = inf;
        while count <=11
            [mask, ~, g_t] = guided_JBF(double(mask), fg_c(:,:,1), -1, count, weight_o,3,5,0.3);%guided imdilate
            if g_t <= g_thres
                disp(['frame ',int2str(i), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;  
        end
        I(:,:,1) = mat2gray(mask)*255;
        if debug_mode, figure(61),imshow(uint8(I)),title('mask after JBF'); end;  
        %%=======================================================
        count = 0;%第一次执行
        weight_i = zeros(1,1);
        %先执行一次全局的guided_JBF，得到完整的weight_o
        [~, weight_o,~]= guided_JBF(double(mask), fg_c(:,:,1),1,count, weight_i,3,5,0.6);%这里只是计算了weight_o
        count = count + 1;  g_thres = 0;%guided thres------per pixel     
        g_t = inf;
        while 1
            [mask, ~, g_t] = guided_JBF(double(mask), fg_c(:,:,1), -1, count, weight_o,3,5,0.6);%guided imdilate
            if g_t <= g_thres
                disp(['frame ',int2str(i), '------------total for ', int2str(count), ' times!']);
                break;
            end
            disp(['g_t = ',int2str(g_t), ', now is ' ,int2str(count), 'th circle time']);
            count = count + 1;
        end
        I(:,:,1) = mat2gray(mask)*255;
        if debug_mode,figure(62),imshow(uint8(I)),title(['frame ', int2str(i), '----res + mask\_d4c+JBF']);end
%         imwrite(uint8(I), [root_dir 'output\vis\res_' int2str(i) '.png']);
%         imwrite(mask, [root_dir 'output\res_' int2str(i) '.png']);
    end
end