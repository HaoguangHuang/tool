%%以fusionForegroundSeg帧为单位，单位内的帧进行融合
function fusionedForegroundData = fusionForegroundFunc1(foregroundData, fusionForegroundSeg)
    frameNum = size(foregroundData,2);
    if mod(frameNum, fusionForegroundSeg) ~= 0
        disp('fusionForegroundFunc error: frameNum不整除fusionForegroundSeg');
        return;
    end
    unitNum = frameNum / fusionForegroundSeg;
    result = {};

    for j = 1:unitNum
        result(j).data = zeros(size(foregroundData(1).data));
        count_map = zeros(size(foregroundData(1).data));
        for i = (fusionForegroundSeg*(j - 1)+1):(fusionForegroundSeg*j)
            Di = double(foregroundData(i).data); %current frame
            nonzero_index = Di > 0;
            result(j).data(nonzero_index) = result(j).data(nonzero_index) + Di(nonzero_index);
            count_map = count_map + nonzero_index;
        end
        nonzero_fg = count_map > 0;
        result(j).data(nonzero_fg) = result(j).data(nonzero_fg) ./count_map(nonzero_fg);
    end

    fusionedForegroundData = result;
end