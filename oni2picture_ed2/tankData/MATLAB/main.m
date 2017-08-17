%% 提取depthMap前景的tank
% clear all; close all;
backgroundFile = 'E:\dataSet\Wajueji_2\background\';
fusionBackgroundFraNum = 100; %融合背景的帧数
fusionForegroundSeg = 50; %前景中，每 x 帧融合得到新的深度图
foregroundFraNum = 50;
backgroundData = {}; %save raw data from background file
foregroundData = {}; %save raw data from foreground file
fusionedBackgroundData = {};
fusionedForegroundData = {};
extractedTankData = {};

module1 = 0; %fusion background
module2 = 1; %fusion foreground
module3 = 1; %extract tank in foreground


%% extract object from color map
% main_color;

if module1 ==1 
    %%read backGround data
    for i = 1:fusionBackgroundFraNum
        backgroundData(i).data = imread([backgroundFile,'depth_',int2str(i),'.png']);
    end
    fusionedBackgroundData = fusionBackgroundFunc1(backgroundData);
    
    imwrite(uint16(fusionedBackgroundData),'E:\dataSet\Wajueji_2\processedData\depth\fusionedBackgroundData\fusionedBackgroundData.png');
end

for k = 81:2:157%1:200
foregroundFile = ['E:\dataSet\Wajueji_2\foreground',int2str(k)];
if module2 == 1
    for i = 1:foregroundFraNum
        foregroundData(i).data = imread([foregroundFile,'depth_',int2str(i),'.png']);
    end
    fusionedForegroundData = fusionForegroundFunc1(foregroundData, fusionForegroundSeg);
    %%save fusionedForegroundData
    for i = 1:foregroundFraNum/fusionForegroundSeg
        imwrite(uint16(fusionedForegroundData(i).data),['E:\dataSet\Wajueji_2\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png'])
    end
end
 
if module3 == 1
    if(module1 == 0) 
        fusionedBackgroundData = imread('E:\dataSet\Wajueji_2\processedData\depth\fusionedBackgroundData\fusionedBackgroundData.png');
    end
    if(module2 == 0)
        for i = 1:foregroundFraNum/fusionForegroundSeg
            fusionedForegroundData(i).data = imread(['E:\dataSet\Wajueji_2\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png']);
        end
    end
    
    ycbcr_mat = imread(['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',int2str(k),'.png']);
    
%     saveDepthMask(fusionedBackgroundData, fusionedForegroundData, k);
    extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData, ycbcr_mat, k);
 
    for i = 1:foregroundFraNum/fusionForegroundSeg
    imwrite(uint16(extractedTankData(i).data), ['E:\dataSet\Wajueji_2\processedData\depth\extractedTankData\extractedTankData',int2str(k),'.png']);
    end
end

disp(['processing k=',int2str(k)]);
end

%% depth recovery
% depthRecovery;

