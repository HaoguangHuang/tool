function transformRGB2YCBCR
%     seriesNum = 200;
%     frame_inSeries = 50;
%     outputFile_address = 'E:\dataSet\Wajueji_2\ycbcr\';
%     inputFile_address = 'E:\dataSet\Wajueji_2\';
%     %%read raw rgb data
%     for i = 1:seriesNum
%         for j = 1:frame_inSeries
%             disp(['processing series ', int2str(i), '  frame', int2str(j)]);
%             raw_rgb_map = imread([inputFile_address, 'foreground', int2str(i), 'color_', int2str(j), '.png']);
%             %%rgb2ycbcr
%             ycbcr_map = rgb2ycbcr(raw_rgb_map);
% %             figure, imshow(ycbcr_map, []);
%             imwrite(ycbcr_map, [outputFile_address, 'ycbcr_foreground', int2str(i), 'color_', int2str(j), '.png']);
%         end
%     end


%% transform color background
    outputFile_address = 'E:\dataSet\Wajueji_2\ycbcr\background\';
    inputFile_address = 'E:\dataSet\Wajueji_2\background\';
    for i = 1:100
        disp(['processing frame', int2str(i)]);
         raw_rgb_map = imread([inputFile_address, 'color_', int2str(i), '.png']);
         ycbcr_map = rgb2ycbcr(raw_rgb_map);
         imwrite(ycbcr_map, [outputFile_address, 'ycbcr_color_', int2str(i), '.png'])
    end
end