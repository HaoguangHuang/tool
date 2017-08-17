for k = 1:2:200
    a = imread(['E:\dataSet\Wajueji_2\processedData\mask_d_c\vis_mask',int2str(k),'_d_c_gd50gc50.png']);
    imwrite(a,['E:\dataSet\Wajueji_2\processedData\mask_d_c\vis\vis_mask',int2str(k),'_d_c_gd50gc50.png'])
    disp(k);
end

