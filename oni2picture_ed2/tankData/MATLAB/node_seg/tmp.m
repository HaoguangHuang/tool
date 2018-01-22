% warped_pc_137 = pcread('./output/pcd_fromMatlab/warped_pc_137.pcd');
% d_138 = imread('./input/Wajueji_2/dRecvy_use_new_guide2/d_137.png');
% camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
% pc_138 = transformUVD2XYZ(d_138,camera_para);
% 
% warped_pc_138 = pcread('./output/pcd_fromMatlab/warped_pc_138.pcd');
% 
% pc_137_138 = [warped_pc_137.Location;pc_138.Location];
% pc_137_138 = pointCloud(pc_137_138);
% pc_137_138_denoise = pcdenoise(pc_137_138);
% pc_137_138_downsample = pcdownsample(pc_137_138, 'gridAverage', 3);
% figure(1),pcshow(pc_137_138),title('pc\_137\_138');
% figure(3),pcshow(pcdenoise(pc_137_138)),title('pc\_137\_138\_pcdenoise');
% figure(4),pcshow(pcdownsample(pc_137_138, 'gridAverage', 3)),title('pc\_137\_138\_pcdownsample');
% figure(2),pcshow(warped_pc_138),title('warped\_pc\_138');
% 
% figure(5),pcshow(pc1),title('pc1');
% figure(6),pcshow(warped_pc),title('warped_pc');


%test how zbuffer+transformationMap work
pc = pcread('./output/pcd_InfiniTAM/20180116_1632/fusioned_pc_132.pcd');
figure(15),pcshow(pc);




function p = transformUVD2XYZ(d, c_pa)
    [H, W] = size(d);
    d = double(d);
    p_array(:,3) = reshape(d,H*W,1);        %z can obtain from var d 
    for u = 1:W
        for v = 1:H
            p_array((u-1)*H+v,1) = (u - c_pa.cx) * d(v,u)/ c_pa.fx;
            p_array((u-1)*H+v,2) = (v - c_pa.cy) * d(v,u)/ c_pa.fy;
        end
    end
    index = p_array(:,3)>0;
 
    p_array = p_array(index,:); 
    p = pointCloud(p_array);
end