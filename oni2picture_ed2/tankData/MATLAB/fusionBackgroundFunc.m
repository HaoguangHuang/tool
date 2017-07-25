function fusionedBackgroundData = fusionBackgroundFunc(backgroundData)
    frameNum = size(backgroundData, 2);
    result = [];
    %以像素为单位作处理
    for u = 1:640
        for v = 1:480
            tmp = 0;
            count = 0;
            for i = 1:frameNum
                if backgroundData(i).data(v,u)~=0
                    tmp = tmp + backgroundData(i).data(v,u);
                    count = count + 1;
                end
            end
            if count == 0   %像素点深度值恒为0
                result(v,u) = 0; continue;
            else
                result(v,u) = double(tmp)/count;
            end
        end
    end
    fusionedBackgroundData = result;
end