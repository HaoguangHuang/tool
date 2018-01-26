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


%================test pcdenoise==================
% pc = pcread('./output/pcd_InfiniTAM/fusioned_pc_51.pcd');
% pc.Color = repmat(uint8([0,0,255]),pc.Count,1);
% figure(15),pcshow(pc),title(sprintf('original pc, pts=%d',pc.Count));
% [pc_d1, in_idx1, out_idx1] = pcdenoise(pc);
% pc_out1 = select(pc,out_idx1); pc_out1.Color = repmat(uint8([255,0,0]),pc_out1.Count,1);
% pc_in1 = select(pc,in_idx1);
% figure(16),pcshow(pc_d1),hold on;
% pcshow(pc_out1,'MarkerSize',10);
% title(sprintf('pcdenoise(pc), pts=%d',pc_d1.Count));
% hold off;
% figure(18),pcshow(pc_in1),title(sprintf('inlier of pcdenoise, pts=%d',pc_in1.Count));
% 
% NumNeighbors = 2;
% thres = 0.5; %mm
% 
% [pc_d2, in_idx2,out_idx2]= pcdenoise(pc,'NumNeighbors',NumNeighbors,'Threshold',thres);
% pc_out2 = select(pc,out_idx2); pc_out2.Color = repmat(uint8([255,0,0]),pc_out2.Count,1);
% pc_in2 = select(pc, in_idx2);
% figure(17),pcshow(pc_d2); hold on;
% pcshow(pc_out2,'MarkerSize',10);
% title(sprintf('NumNeighbors=%d, Threshold=%d,pts=%d',NumNeighbors,thres,...
%     pc_d2.Count));
% hold off;
% figure(19),pcshow(pc_in2),title(sprintf('inlier of pcdenoise2, pts=%d',pc_in2.Count));

%================test radius filter of pcdownsample===============
pc = pcread('./output/pcd_InfiniTAM/fusioned_pc_51.pcd');
pc.Color = repmat(uint8([0,0,255]),pc.Count,1);
r = 5;
pc_gf = pcdownsample(pc,'gridAverage',r);

step = ceil(pc.Count/50000);
idx = 1:step:pc.Count;
pc_stepDowns = select(pc,idx);
figure(21),pcshow(pc_gf),title(sprintf('grid Average, r =%d ,inlier pts=%d',r,pc_gf.Count));
figure(22),pcshow(pc_stepDowns),title(sprintf('step downsample, step=%d, inlier pts=%d',step,pc_stepDowns.Count));



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