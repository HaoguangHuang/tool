function fusionedBackgroundData = fusionBackground_color_Func(backgroundData)
    %%fusion the 30-100 background frame
    frameNum = size(backgroundData, 2);
    background = zeros(480,640);
    count_map = zeros(size(background));
    fusionedBackgroundData = zeros(size(backgroundData(1).data));%480*640*3
    fusionedBackgroundData_y = zeros(size(background));
    fusionedBackgroundData_cb = zeros(size(background));
    fusionedBackgroundData_cr = zeros(size(background));
    %% channel y
    for i = 1:frameNum
        Yi = double(backgroundData(i).data(:,:,1));  %now the backgroundData(i) is in ycbcr format
        nonzero_index = Yi > 0;
        background(nonzero_index) = background(nonzero_index) + Yi(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedBackgroundData_y(nonzero_bg) = background(nonzero_bg) ./ count_map(nonzero_bg);
     fusionedBackgroundData(:,:,1) = fusionedBackgroundData_y;
    
    %% channel cb
    background = zeros(480,640);
    count_map = zeros(size(background));
    for i = 1:frameNum
        CBi = double(backgroundData(i).data(:,:,2));  %now the backgroundData(i) is in ycbcr format
        nonzero_index = CBi > 0;
        background(nonzero_index) = background(nonzero_index) + CBi(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedBackgroundData_cb(nonzero_bg) = background(nonzero_bg) ./ count_map(nonzero_bg);
    fusionedBackgroundData(:,:,2) = fusionedBackgroundData_cb;
    
    %% channel cr
    background = zeros(480,640);
    count_map = zeros(size(background));
    for i = 1:frameNum
        CRi = double(backgroundData(i).data(:,:,3));  %now the backgroundData(i) is in ycbcr format
        nonzero_index = CRi > 0;
        background(nonzero_index) = background(nonzero_index) + CRi(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedBackgroundData_cr(nonzero_bg) = background(nonzero_bg) ./ count_map(nonzero_bg);
    fusionedBackgroundData(:,:,3) = fusionedBackgroundData_cr;
end