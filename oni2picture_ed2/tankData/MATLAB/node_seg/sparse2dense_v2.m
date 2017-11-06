%% SPARSE2DENSE_V2
%======JTF插correspondence=====
% 以第一帧图为guidance，在图二中插值出图一中没有unique_corr的点的correspondence
%         pc1:pointCloud.      guidance点云，需要先转到pc2的坐标，然后从3D投影到2D相机平面
%         pc2:pointCloud.      被操作的点云，需要从3D投影到2D相机平面
% point_corr:pc1.Count*3.     [indice, correspondence, dist]
% camera_para:struct.          containing fx, fy, cx, cy
%    densePC2:640*480 array.   插值后稠密的pointCloud1的深度图  
%        Tmat:cell.            save ICP result of each node in each layers
% pc_bestNode_distr: array.    n*1.  记录每个point归属的最佳node是哪个
%    idx_map2:插值出的稠密D2上的像素点对应pc1中的corr下标
%   denseMap2:插值出的稠密D2
%
% Note:思路是以图一为guidance，操作图二，进行三边guided JBF。步骤如下：
%      1.pc1中有unique_corr以及没有unique_corr的点，分别transform到pc2坐标上，得到mask1和mask0.
%      2.mask = (~mask1) & mask0. 后续操作mask==1的点，此处不考虑z-buffer
%      3.tripple_filter中，只对方形窗口中depth~=0的深度点进行加权平均。此处要求所有mask==1的像素点都要获得深度值
%      tripple_filter
%      a) 第一帧图中的某点 i (该点不拥有unique_correspondence)作为中心， 以width为边长构造方形窗口
%      b) 窗口中像素点的权值为: w = w_intensity * w_distance * w_depth
%      c) 点 i 的深度值为 d_i = Σα_j * d_j / Σα_j

function [denseMap2, idx_map2]= sparse2dense_v2(pc1, pc2, point_corr, camera_para, Tmat, pc_bestNode_distr)
    if nargin < 1, load('pc.mat','pc_197'); pc1 = pc_197; clear pc_197; end;
    if nargin < 2, load('pc.mat','pc_198'); pc2 = pc_198; clear pc_198; end;
    if nargin < 3, load('corrIndex.mat','corrIndex_wc03_thres2');
        point_corr = corrIndex_wc03_thres2;  clear corrIndex_wc03_thres2; 
    end
    if nargin < 4, camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202); end
    if nargin < 5, load('Tmat_wc03.mat'); end
    if nargin < 6, load('pc_bestNode_distr.mat');
        pc_bestNode_distr = pc_bestNode_distr_wc03; clear pc_bestNode_distr_wc03; 
    end
    %%======把pc1的unique_corr投影到pc2=======
    pc1_inCoo2 = transform_pc1_into_coordinate_of_pc2(pc1, Tmat, pc_bestNode_distr);
    
    %%======投影经过POI滤波的pc1_inCoo2到相机平面======
    unique_corr = point_corr(point_corr(:,2)>0,:);
    not_unique_corr = point_corr(point_corr(:,2)==0,:);
    
    xyz_pc0 = select(pc1_inCoo2, not_unique_corr(:,1));  %之前匹配的corr间的距离超过2mm
    y_pc0 = xyz_pc0.Color(:,1); xyzidx_pc0 = [xyz_pc0.Location(:,:), not_unique_corr(:,1)];
    %D0:pc1_inCoo2中所有没有unique_correspondence的点的投影uvd
    %Y0:pc1_inCoo2中所有没有unique_correspondence的点的投影uvY
    [~, D0, Y0, idx_map0]= transformXYZ2UVDI(xyzidx_pc0, camera_para, y_pc0, not_unique_corr);
    mask0 = D0 > 0;
    
    xyz_pc1 = select(pc1_inCoo2, unique_corr(:,1));  %20342  
    y_pc1 = xyz_pc1.Color(:,1); xyzidx_pc1 = [xyz_pc1.Location(:,:), unique_corr(:,1)]; 
    
    %D1:pc1_inCoo2中所有拥有unique_correspondence的点的投影uvd
    %Y1:pc1_inCoo2中所有拥有unique_correspondence的点的投影uvY
    [~, D1, Y1, ~]= transformXYZ2UVDI(xyzidx_pc1, camera_para, y_pc1, unique_corr);
    mask1 = D1 > 0;  %20280
    
    %D2:pointCloud2中所有拥有unique_correspondence的点的投影uvd
    %Y2:pointCloud2中所有拥有unique_correspondence的点的投影uvY
    %idx_map2:记录D2中投影点(都有unique_corr)在pc1中的对应corr
    xyz_pc2 = select(pc2,unique_corr(:,2));
    y_pc2 = xyz_pc2.Color(:,1); xyzidx_pc2 = [xyz_pc2.Location(:,:), unique_corr(:,2)];
    [~, D2, Y2, idx_map2]= transformXYZ2UVDI(xyzidx_pc2, camera_para, y_pc2, unique_corr);
    
    %mask>0的点是要tripple_filter处理的点
    mask = (~mask1) & mask0 & (D2 == 0);
    mask_idx_map = double(mask) .* idx_map0;
    
    [denseMap2, addMap_idx_map] = tripple_guided_JBF(D1, D2, Y1, Y2, mask, mask_idx_map);  %denseMap是稠密的D2
   
    
    %%======合并addMap_idx_map和idx_map2。======
    %合并发生像素重叠的话，idx_map2的优先级高于addMap
    if sum(sum((addMap_idx_map>0)&(idx_map2>0)))~=0, error('idx_map && addMap ~= 0');end  %检查是否有像素重叠
    idx_map2 = addMap_idx_map + idx_map2;
    
    %检查idx_map中的点是否都有pc_best_Node——没问题
%     load('test.mat','idx_map');
%     idx_map = int16(idx_map);
%     for r = 1:480
%         for c = 1:640
%             if idx_map(r,c) == 0, continue; end
%             if pc_bestNode_distr(idx_map(r,c)) ~= 0
%                 disp(['r=',num2str(r),', c=',num2str(c)]);
%             end
%         end
%     end
end


function [vud_pc, D, Y, idx_map]= transformXYZ2UVDI(xyzidx_pc, camera_para, y_pc1, unique_corr)
    D = zeros(480,640);  Y = zeros(480,640);
    xyz_pc = xyzidx_pc(:,1:3); idx = xyzidx_pc(:,4);
    vud_pc = zeros(size(xyz_pc));
    vud_pc(:,3) = xyz_pc(:,3);                                                       %d = z
    count = 0;
    idx_map = zeros(size(D));
    for i = 1:size(xyz_pc,1)
        if vud_pc(i,3) == 0, continue; end
        vud_pc(i,2) = round(xyz_pc(i,1)*camera_para.fx/xyz_pc(i,3)+camera_para.cx);  %u = (x*f_x)/z + c_x
        vud_pc(i,1) = round(xyz_pc(i,2)*camera_para.fy/xyz_pc(i,3)+camera_para.cy);  %v = (y*f_y)/z + c_y
        D(vud_pc(i,1),vud_pc(i,2)) = vud_pc(i,3);
        Y(vud_pc(i,1),vud_pc(i,2)) = y_pc1(i,1);
        idx_map(vud_pc(i,1),vud_pc(i,2)) = unique_corr(i,1);
        count = count + 1;
    end
%     figure(1),imshow(D,[]);
%     figure(2),imshow(idx_map,[]);
end

%% TRANSFORM_PC1_INTO_COORDINATE_OF_PC2
function pc1_inCoo2 = transform_pc1_into_coordinate_of_pc2(pc1, Tmat, pc_bestNode_distr)
    xyz_pc1_inCoo2 = zeros(pc1.Count,3);
    for i = 1:pc1.Count
        if pc_bestNode_distr(i,1) == 0, continue; end;  %exclude the poins that belonging to no node 
        p = pc1.Location(i,:);
        T = Tmat{4}{pc_bestNode_distr(i,1)};
        p_afterTran = transformPointsForward(T, p);
        xyz_pc1_inCoo2(i,:) = p_afterTran;
    end
    pc1_inCoo2 = pointCloud(xyz_pc1_inCoo2);
    pc1_inCoo2.Color = pc1.Color;
end







