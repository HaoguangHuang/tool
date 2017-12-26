ptCloud = pcread('/home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/Files/wajueji/output/hhg.pcd');
figure(1),pcshow(ptCloud);xlabel('x'),ylabel('y'),zlabel('z');

ptCloud1 = pcread('/home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/node_seg/output/pointcloud/pc_1.pcd');
figure(2),pcshow(ptCloud1),title('original');xlabel('x'),ylabel('y'),zlabel('z');




load pc.mat;
pc2 = pc_198;
pc2.Color = repmat(uint8([0,0,255]),pc2.Count,1);
ptCloud = pcread('/home/hhg/Documents/myGithub2/InfiniTAM_v2_hhg/InfiniTAM/Files/wajueji/output/hhg.pcd');
ptCloud.Color = repmat(uint8([255,0,0]),ptCloud.Count,1);
figure(13),pcshow(pc2);hold on; pcshow(ptCloud),hold off;
xlabel('x'),ylabel('y'),zlabel('z');