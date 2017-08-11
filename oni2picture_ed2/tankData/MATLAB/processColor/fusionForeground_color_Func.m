%% 融合color map（）
function fusionedForegroundData = fusionForeground_color_Func(foregroundData, fusionForegroundSeg)
    frameNum = size(foregroundData, 2);   %40
    if mod(frameNum, fusionForegroundSeg) ~=0
        disp('fusionForeground_color_Func不能整除');
    end
    
    foreground = zeros(480,640);
    count_map = zeros(size(foreground));
    fusionedForegroundData = zeros(size(foregroundData(1).data));%480*640*3
    fusionedForegroundData_y = zeros(size(foreground));
    fusionedForegroundData_cb = zeros(size(foreground));
    fusionedForegroundData_cr = zeros(size(foreground));
    %% channel y
   for i = 1:frameNum
        Yi = double(foregroundData(i).data(:,:,1));  %now the foregroundData(i) is in ycbcr format
        nonzero_index = Yi > 0;
        foreground(nonzero_index) = foreground(nonzero_index) + Yi(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedForegroundData_y(nonzero_bg) = foreground(nonzero_bg) ./ count_map(nonzero_bg);
     fusionedForegroundData(:,:,1) = fusionedForegroundData_y;
    
    %% channel cb
    foreground = zeros(480,640);
    count_map = zeros(size(foreground));
    for i = 1:frameNum
        CBi = double(foregroundData(i).data(:,:,2));  %now the foregroundData(i) is in ycbcr format
        nonzero_index = CBi > 0;
        foreground(nonzero_index) = foreground(nonzero_index) + CBi(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedForegroundData_cb(nonzero_bg) = foreground(nonzero_bg) ./ count_map(nonzero_bg);
    fusionedForegroundData(:,:,2) = fusionedForegroundData_cb;
     
    %% channel cr
    foreground = zeros(480,640);
    count_map = zeros(size(foreground));
    for i = 1:frameNum
        CRi = double(foregroundData(i).data(:,:,3));  %now the foregroundData(i) is in ycbcr format
        nonzero_index = CRi > 0;
        foreground(nonzero_index) = foreground(nonzero_index) + CRi(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedForegroundData_cr(nonzero_bg) = foreground(nonzero_bg) ./ count_map(nonzero_bg);
    fusionedForegroundData(:,:,3) = fusionedForegroundData_cr;
    
end