function extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData)
    fusionedForeFraNum = size(fusionedForegroundData, 2);
    thres = 30;%mm
    result = {};
    %%前景减背景
    for i = 1:fusionedForeFraNum
%         judgeArr =  abs((uint16(fusionedBackgroundData) - (uint16(fusionedForegroundData(i).data))))>thres;
        judgeArr =  abs((double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data))))>thres;
        result(i).data = uint16(fusionedForegroundData(i).data) .* uint16(judgeArr);
        
        %%转化到[0-255]
        result(i).data = double(result(i).data) .* (255/10000);
        
    end
    extractedTankData = result;
    
    %%visualization
    for i = 1:1%fusionedForeFraNum
        figure; imshow(extractedTankData(i).data, []); title(['第',int2str(i),'帧']);
    end
end