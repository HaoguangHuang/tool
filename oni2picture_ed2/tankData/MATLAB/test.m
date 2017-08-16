for i = 1:60
    mask(i).data = imread(['E:\dataSet\Wajueji_2\processedData\finalMask\mask',int2str(i),'.png']);
% mask(i).data = imread(['E:\dataSet\Wajueji_2\processedData\intensityMask\mask',...
%         int2str(i),'_d.png']);
        fg(i).data = imread(['E:\dataSet\Wajueji_2\processedData\color\fusionedForegroundData\fusionedForegroundData',...
        int2str(i),'.png']);
 
    %%guided filter
    count = 0;
     weight_i = zeros(1,1);
     fff = fg(i).data(:,:,1);
    [mask_1, ~,~] = guided_JBF(mask(i).data>0, fff, 0,count, weight_i);%
    [mask_1, ~,~] = guided_JBF(mask_1, fff, 0,count, weight_i);
    [mask_1, ~,~] = guided_JBF(mask_1, fff, 0,count, weight_i);
    [mask_1, ~,~] = guided_JBF(mask_1, fff, 0,count, weight_i);
    [mask_jbf, ~,~] = guided_JBF(mask_1, fff, 0,count, weight_i);
    
    I(:,:,1) = uint8(mask_jbf) *255;
    I(:,:,2) = uint8(fg(i).data(:,:,1));
    I(:,:,3) = zeros(size(mask(1).data));
    figure(113),imshow(uint8(I),[]),title(['mask after , frame',int2str(i)]);
    I(:,:,1) = uint8(mask(i).data) * 255;
    figure(114),imshow(uint8(I),[]),title(['mask before , frame',int2str(i)]);
end

