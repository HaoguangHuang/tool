%% 提取depthMap前景的tank
% clear all; close all;
for k = 1:24
% backgroundFile = 'E:\dataSet\set9\background\';
backgroundFile = 'E:\dataSet\Wajueji_1\background\';
% foregroundFile = 'E:\dataSet\set9\foreground\';
foregroundFile = ['E:\dataSet\Wajueji_1\foreground',int2str(k),'\'];
fusionBackgroundFraNum = 100; %融合背景的帧数
fusionForegroundSeg = 100; %前景中，每 x 帧融合得到新的深度图
foregroundFraNum = 100;
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
%         backgroundData(i).data = imread([backgroundFile,'depth_output',int2str(i),'.png']);
        backgroundData(i).data = imread([backgroundFile,'depth_',int2str(i),'.png']);
    end
    fusionedBackgroundData = fusionBackgroundFunc1(backgroundData);
    
%     imwrite(uint16(fusionedBackgroundData),'E:\dataSet\set9\processedData\depth\fusionedBackgroundData\fusionedBackgroundData.png');
    imwrite(uint16(fusionedBackgroundData),'E:\dataSet\Wajueji_1\processedData\depth\fusionedBackgroundData\fusionedBackgroundData.png');
    
end

if module2 == 1
    for i = 1:foregroundFraNum
%         foregroundData(i).data = imread([foregroundFile,'depth_output',int2str(i),'.png']);
        foregroundData(i).data = imread([foregroundFile,'depth_',int2str(i),'.png']);
    end
%     fusionedForegroundData = fusionForegroundFunc(foregroundData, fusionForegroundSeg);
    fusionedForegroundData = fusionForegroundFunc1(foregroundData, fusionForegroundSeg);
    %%save fusionedForegroundData
    for i = 1:foregroundFraNum/fusionForegroundSeg
%         imwrite(uint16(fusionedForegroundData(i).data),['E:\dataSet\set9\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png']);
        imwrite(uint16(fusionedForegroundData(i).data),['E:\dataSet\Wajueji_1\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png'])
    end
end
 
if module3 == 1
    if(module1 == 0) 
%         fusionedBackgroundData = imread('E:\dataSet\set9\processedData\depth\fusionedBackgroundData\fusionedBackgroundData.png');
        fusionedBackgroundData = imread('E:\dataSet\Wajueji_1\processedData\depth\fusionedBackgroundData\fusionedBackgroundData.png');
    end
    if(module2 == 0)
        for i = 1:foregroundFraNum/fusionForegroundSeg
%             fusionedForegroundData(i).data = imread(['E:\dataSet\set9\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png']);
            fusionedForegroundData(i).data = imread(['E:\dataSet\Wajueji_1\processedData\depth\fusionedForegroundData\fusionedForegroundData',int2str(i),'.png']);
        end
    end
    extractedTankData = extractTankFunc(fusionedBackgroundData, fusionedForegroundData);
    %save extractedTankData
    for i = 1:foregroundFraNum/fusionForegroundSeg
%         imwrite(uint8(extractedTankData(i).data), ['E:\dataSet\set9\processedData\depth\extractedTankData\extractedTankData',int2str(i),'.png']);
    imwrite(uint16(extractedTankData(i).data), ['E:\dataSet\Wajueji_1\processedData\depth\extractedTankData\extractedTankData',int2str(k),'.png']);
    end
end

disp(['processing k=',int2str(k)]);
end
%% 提取颜色
% main_color;


