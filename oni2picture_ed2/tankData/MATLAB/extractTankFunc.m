function extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData)
    fusionedForeFraNum = size(fusionedForegroundData, 2);
    thres = 60;%mm
    result = {};
    %%前景减背景
    
    for i = 1:fusionedForeFraNum
%         judgeArr =  abs((uint16(fusionedBackgroundData) - (uint16(fusionedForegroundData(i).data))))>thres;
        mask1 =  (double(fusionedBackgroundData) - (double(fusionedForegroundData(i).data)))>thres;
        
        mask2 = fusionedForegroundData(i).data < 1000;
%         mask2 = ones(size(mask1));
        
        result(i).data = uint16(fusionedForegroundData(i).data) .* uint16(mask1) .* uint16(mask2);
        se = strel('rectangle', [5 5]);        
        mask3 = imdilate(result(i).data > 0, se);
        mask3 = imerode(mask3, se);
 
%         figure, hold on;        
%         subplot(1,3,1);
%         imshow(result(i).data,[]);          
%         subplot(1,3,2); imshow(mask3,[]);
%         subplot(1,3,3); imshow(mask3,[]);
%         hold off;
        
        result(i).data = result(i).data .* uint16(mask3);
        %%转化到[0-255]
%          result(i).data = double(result(i).data) .* (255/10000);
        
    end
    
    extractedTankData = result;
    
    %%visualization
%     for i = 1:fusionedForeFraNum
%         figure; imshow(extractedTankData(i).data, []); title(['第',int2str(i),'帧']);
%     end
end