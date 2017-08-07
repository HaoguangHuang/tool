function fusionedBackgroundData = fusionBackgroundFunc1(backgroundData)
    frameNum = size(backgroundData, 2);
    result = zeros(size(backgroundData(1).data));
    count_map = zeros(size(result));
    fusionedBackgroundData = zeros(size(result));
    for i = 1:frameNum
        Di = double(backgroundData(i).data);
        nonzero_index = Di > 0;
        result(nonzero_index) = result(nonzero_index) + Di(nonzero_index);
        count_map = count_map + nonzero_index;
    end
    nonzero_bg = count_map > 0;
    fusionedBackgroundData(nonzero_bg) = result(nonzero_bg) ./ count_map(nonzero_bg);
end
    