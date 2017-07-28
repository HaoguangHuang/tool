function extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData)
    fusionedForeFraNum = size(fusionedForegroundData, 2);
    thres = 300;%mm
    result = {};
    %%Ç°¾°¼õ±³¾°
    for i = 1:fusionedForeFraNum
%         judgeArr =  abs((uint16(fusionedBackgroundData) - (uint16(fusionedForegroundData(i).data))))>thres;
%         judgeArr =  abs((double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data))))>thres;
        judgeArr1 =  (double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data)))<-200;
        judgeArr2 = (double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data)))>-300;
        judgeArr = judgeArr1 .* judgeArr2;
        result(i).data = uint16(fusionedForegroundData(i).data) .* uint16(judgeArr);
    end
    extractedTankData = result;
    
    %%visualization
    for i = 1:fusionedForeFraNum
        figure; imshow(extractedTankData(i).data, []); title(['µÚ',int2str(i),'Ö¡']);
    end
end