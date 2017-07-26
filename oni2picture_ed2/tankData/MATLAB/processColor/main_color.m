%% 提取colorMap的前景
clear all; close all;
backgroundFile = 'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\background\colormap\';
foregroundFile = 'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\foreground\colormap\';
extractedDepthFile = 'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\processedDepthMap\extractedTankData\';
fusionBackgroundFraNum = 30; %彩色图不需要背景融合
fusionForegroundSeg = 5; %前景中，每 x 帧融合得到新的彩色图
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
    %%read backGround data
    for i = 1:fusionBackgroundFraNum
        %backgroundData(i).data = imread([backgroundFile,'depth_output',int2str(i),'.png']);
        backgroundData(i).data = imread([backgroundFile,'color','30','.png']);
    end
    %fusionedBackgroundData = fusionBackground_color_Func(backgroundData);
    fusionedBackgroundData = backgroundData(1).data;
    imwrite(uint8(fusionedBackgroundData),'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\processedColorMap\fusionedBackgroundData\fusionedBackgroundData.png');
end

if module2 == 1
    for i = 1:foregroundFraNum
        foregroundData(i).data = imread([foregroundFile,'color',int2str(i),'.png']);
    end
    
    %%只是简单地进行隔帧采样
    fusionedForegroundData = fusionForeground_color_Func(foregroundData, fusionForegroundSeg);
    
    %%save fusionedForegroundData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint8(fusionedForegroundData(i).data),['E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\processedColorMap\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png'])
    end
end

if module3 == 1
    if(module1 == 0) 
        fusionedBackgroundData = imread('E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\processedColorMap\fusionedBackgroundData\fusionedBackgroundData.png');
    end
    if(module2 == 0)
        for i = 1:foregroundFraNum/fusionForegroundSeg
            fusionedForegroundData(i).data = imread(['E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\processedColorMap\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png']);
        end
    end
    
    %%read extracted depthMap
    for i = 1:foregroundFraNum/fusionForegroundSeg
        extractedDepthMap(i).data = imread([extractedDepthFile,'extractedTankData',int2str(i),'.png']);
    end
    
    extractedTankData = extractTank_color_Func(fusionedBackgroundData, fusionedForegroundData, extractedDepthMap);
    
    %%save extractedTankData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint8(extractedTankData(i).data), ['E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\processedData\processedColorMap\extractedTankData\extractedTankData',int2str(i),'.png']);
    end
end

