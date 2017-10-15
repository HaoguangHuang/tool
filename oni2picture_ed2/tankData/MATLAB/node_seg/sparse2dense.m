%% SPARSE2DENSE
% 以第一帧图为guidance，把图二的sparse unique correspondence point cloud插值成dense point cloud
%         pc1:pointCloud.      被操作的点云，需要从3D投影到2D相机平面
%         D2 :640*480 array.   pointCloud2的原深度图
% unique_corr:pc1.Count*3.     [indice, correspondence, dist]
% camera_para:struct.          containing fx, fy, cx, cy
%        Yuv1:640*480*3 array
%        Yuv2:640*480*3 array
%    densePC1:640*480 array.   插值后稠密的pointCloud1的深度图  
%
% Note:思路是以图二为guidance，操作图一，进行三边guided JBF。步骤如下：
%      a) 第一帧图中的某点 i (该点不拥有unique_correspondence)作为中心， 以width为边长构造方形窗口
%      b) 窗口中像素点的权值为: w = w_intensity * w_distance * w_depth
%      c) 点 i 的深度值为 d_i = Σα_j * d_j / Σα_j

function denseMap1 = sparse2dense(pc1, D2, unique_corr, camera_para, Yuv1, Yuv2)
    if nargin < 1, load('pc.mat','pc_197'); pc1 = pc_197; clear pc_197; end;
    if nargin < 2, D2 = imread('E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\d_198.png');end
    if nargin < 3, load('corrIndex.mat','corrIndex_wc03_thres2');
        unique_corr = corrIndex_wc03_thres2;  clear corrIndex_wc03_thres2; 
    end
    if nargin < 4, camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202); end
    if nargin < 5, Yuv1 = imread('E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\c_197.png');end
    if nargin < 6, Yuv2 = imread('E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\c_198.png');end
    %%======投影经过POI滤波的pc1到相机平面======
    validIndex = unique_corr(:,2)>0;  %exclude invalid correspondence
    unique_corr = unique_corr(validIndex,:);
    xyz_pc = select(pc1, unique_corr(:,1));  
    xyz_pc1 = xyz_pc.Location(:,:); y_pc1 = xyz_pc.Color(:,1);
    
    %D1:pointCloud1中所有拥有unique_correspondence的点的投影uvd
    %Y1:pointCloud1中所有拥有unique_correspondence的点的投影uvY
    [~, D1, Y1]= transformXYZ2UVD(xyz_pc1, camera_para, y_pc1);  

    denseMap1 = tripple_guided_JBF(D1, D2, Y1, Yuv2(:,:,2));
   
end

%map：返回一张二维的投影图
function [vud_pc, D, Y]= transformXYZ2UVD(xyz_pc, camera_para, y_pc1)
    D = zeros(480,640);  Y = zeros(480,640);
    vud_pc = zeros(size(xyz_pc));
    vud_pc(:,3) = xyz_pc(:,3);                                                       %d = z
    for i = 1:size(xyz_pc,1)
        vud_pc(i,2) = round(xyz_pc(i,1)*camera_para.fx/xyz_pc(i,3)+camera_para.cx);  %u = (x*f_x)/z + c_x
        vud_pc(i,1) = round(xyz_pc(i,2)*camera_para.fy/xyz_pc(i,3)+camera_para.cy);  %v = (y*f_y)/z + c_y
        D(vud_pc(i,1),vud_pc(i,2)) = vud_pc(i,3);
        Y(vud_pc(i,1),vud_pc(i,2)) = y_pc1(i,1);
    end
end








