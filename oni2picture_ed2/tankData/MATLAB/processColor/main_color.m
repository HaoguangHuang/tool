%% 提取colorMap的前景
% clear all; close all;

backgroundFile = 'E:\dataSet\Wajueji_2\ycbcr\background\';

extractedDepthFile = 'E:\dataSet\Wajueji_2\processedData\depth\extractedTankData\';
fusionBackgroundFraNum = 100; %彩色图不需要背景融合
background_start = 30;%剔除前面白平衡失调的帧
fusionForegroundSeg = 40; %前景中，每 x 帧融合得到新的彩色图
foreground_start = 10;
foregroundFraNum = 50;
backgroundData = {}; %save raw data from background file
foregroundData = {}; %save raw data from foreground file
fusionedBackgroundData = {};
fusionedForegroundData = {};
extractedTankData = {};

module1 = 0; %fusion background
module2 = 0; %fusion foreground
module3 = 1; %extract tank in foreground

if module1 ==1 
    %%read backGround data
    for i = (background_start+1):fusionBackgroundFraNum
        backgroundData(i-background_start).data = imread([backgroundFile,'ycbcr_color_',int2str(i),'.png']);
    end
    fusionedBackgroundData = fusionBackground_color_Func(backgroundData);
%     fusionedBackgroundData = backgroundData(1).data;

    imwrite(uint8(fusionedBackgroundData),'E:\dataSet\Wajueji_2\processedData\color\fusionedBackgroundData\fusionedBackgroundData.png');
end

for k = 23:200%143
foregroundFile = ['E:\dataSet\Wajueji_2\ycbcr\ycbcr_foreground',int2str(k)];
if module2 == 1
    for i = (foreground_start+1):foregroundFraNum
        foregroundData(i-foreground_start).data = imread([foregroundFile,'color_',int2str(i),'.png']);
    end
    
    fusionedForegroundData = fusionForeground_color_Func(foregroundData, fusionForegroundSeg);
    
    %%save fusionedForegroundData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint8(fusionedForegroundData),['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png'])
    end
end

if module3 == 1
    if(module1 == 0) 
        fusionedBackgroundData = imread('E:\dataSet\Wajueji_2\processedData\color\fusionedBackgroundData\fusionedBackgroundData.png');
    end
    if(module2 == 0)
        for i = 1:foregroundFraNum/fusionForegroundSeg
            fusionedForegroundData = imread(['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png']);
        end
    end
    
    %%read extracted depthMap
    for i = 1:foregroundFraNum/fusionForegroundSeg
        extractedDepthMap(i).data = imread([extractedDepthFile,'extractedTankData',int2str(k),'.png']);
    end
    
%      saveColorMask(fusionedBackgroundData, fusionedForegroundData, k);
     extractedTankData = extractTank_color_ycbcr_Func(fusionedBackgroundData, fusionedForegroundData, extractedDepthMap, k);
    
    %%save extractedTankData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint8(extractedTankData(i).data), ['E:\dataSet\Wajueji_2\processedData\color\extractedTankData\extractedTankData',int2str(k),'.png']);
    end
end

disp(['processing k=',int2str(k)]);
end
