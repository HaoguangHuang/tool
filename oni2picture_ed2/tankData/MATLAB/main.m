%% 提取前景的tank
backgroundFile = 'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\background\depthmap\';
foregroundFile = 'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\foreground\depthmap\';
fusionBackgroundFraNum = 100; %融合背景的帧数
fusionForegroundSeg = 5; %前景中，每 x 帧融合得到新的深度图
foregroundFraNum = 600;
backgroundData = {}; %save raw data from background file
foregroundData = {}; %save raw data from foreground file
fusionedBackgroundData = {};
fusionedForegroundData = {};
extractedTankData = {};

module1 = 1; %fusion background
module2 = 1; %fusion foreground
module3 = 1; %extract tank in foreground

if module1 ==1 
    %read backGround data
    for i = 1:fusionBackgroundFraNum
        backgroundData(i).data = imread([backgroundFile,'depth_output',int2str(i),'.png']);
    end
    fusionedBackgroundData = fusionBackgroundFunc(backgroundData);
    imwrite(uint16(fusionedBackgroundData),'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\fusionedBackgroundData\fusionedBackgroundData.png');
end

if module2 == 1
    for i = 1:foregroundFraNum
        foregroundData(i).data = imread([foregroundFile,'depth_output',int2str(i),'.png']);
    end
    fusionedForegroundData = fusionForegroundFunc(foregroundData, fusionForegroundSeg);
    %save fusionedForegroundData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint16(fusionedForegroundData(i).data),['E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png'])
    end
end

if module3 == 1
    if(module1 == 0) 
        fusionedBackgroundData = imread('E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\fusionedBackgroundData\fusionedBackgroundData.png');
    end
    if(module2 == 0)
        for i = 1:foregroundFraNum/fusionForegroundSeg
            fusionedForegroundData(i).data = imread(['E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png']);
        end
    end
    extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData);
    %save extractedTankData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint16(extractedTankData(i).data), ['E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\extractedTankData\extractedTankData',int2str(i),'.png']);
    end
end