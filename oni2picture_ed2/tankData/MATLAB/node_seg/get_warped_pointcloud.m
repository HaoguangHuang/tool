%% GET_WARPED_POINTCLOUD
%======JTF:interpolate transformation of points from pc2, which have depth value but no unique_corr=====
% 以第一帧图为guidance，在图二中插值出有深度值但没有unique_corr的点的transformation
%         pc1:pointCloud.      operated pc. Needed to be transformed into coo of pc2, then project from 3D to 2D
%         pc2:pointCloud.      guidance pc. 
% point_corr:pc1.Count*3.     [indice, correspondence, dist]
% camera_para:struct.          containing fx, fy, cx, cy
%        Tmat:cell.            save ICP result of each node in each layers
% pc_bestNode_distr: array.    n*1.  record the best related node of each point in pc1
% warpedPointcloud:pointCloud. point number equals to pc1. 
%
% Note:思路是以图二为guidance，操作图一，进行JBF。步骤如下：
%      1.pc1中有unique_corr的点，forward_project得到D1, Y1, transformationMap1
%      2.pc1中没有unique_corr的点，通过z-buffer分成两类，一类是不被遮挡的点，一类是被遮挡的点
%      3.对不被遮挡的点投影得到的D2，进行JBF插值得到transformation；对被遮挡的点，transformation全部用顶层node的ICP结果。
%      
%      JBF中，只对方形窗口中depth~=0的深度点进行加权平均。此处要求D2所有mask==1的像素点都要获得transformation
%      JBF
%      a) D2中的某点 i (该点不拥有unique_correspondence)作为中心， 以width为边长构造方形窗口
%      b) 窗口中像素点的权值为: w = w_distance * w_depth
%      c) 点 i 的[α,β，γ，t1,t2,t3]为 t_i = Σα_j * t_j / Σα_j
%
%
% Here, status_1 represents points with unique_corr; status_2 represents that without unique_corr but can proj 
%   into 2D plane; status_3 represents that without unique_corr and cannot proj into 2D plane since occlusion 



function warpedPointcloud = get_warped_pointcloud(pc1, pc2, point_corr, camera_para, Tmat, pc_bestNode_distr)
    addpath(genpath('E:\matlab_thirdparty_lib'));
    if nargin < 1, load('pc.mat','pc_197'); pc1 = pc_197; clear pc_197; end
    if nargin < 2, load('pc.mat','pc_198'); pc2 = pc_198; clear pc_198; end
    if nargin < 3, load('corrIndex.mat','corrIndex_wc03_thres2');
        point_corr = corrIndex_wc03_thres2;  clear corrIndex_wc03_thres2; 
    end
    if nargin < 4, camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202); end
    if nargin < 5, load('Tmat_wc03.mat'); end
    if nargin < 6, load('pc_bestNode_distr.mat');
        pc_bestNode_distr = pc_bestNode_distr_wc03; clear pc_bestNode_distr_wc03; 
    end
    
    %%======投影经过POI滤波的pc1_inCoo2到相机平面======
    unique_corr = point_corr(point_corr(:,2)>0,:);
    not_unique_corr = point_corr(point_corr(:,2)==0,:);
    
    
    xyz_pc1_unique = select(pc1, unique_corr(:,1));  %20342  
    y_pc1 = xyz_pc1_unique.Location(:,1);  %not use
    xyzidx_pc1 = [xyz_pc1_unique.Location(:,:), unique_corr(:,1)]; 
    %D1:pc1_inCoo2中所有拥有unique_correspondence的点的投影uvd. guidance
    %Y1:pc1_inCoo2中所有拥有unique_correspondence的点的投影uvY
    %transformationMap1:480*640*6. 记录每个像素点的
    [~, D1, ~, idx_map1]= transformXYZ2UVDI(xyzidx_pc1, camera_para, y_pc1, unique_corr);
    transformationMap1 = idx_to_transformation(idx_map1, pc_bestNode_distr, Tmat);
    
    
    %D2:pointCloud1中所有不拥有unique_correspondence的点的投影uvd. the point not been occluded  
    %Y2:pointCloud1中所有不拥有unique_correspondence的点的投影uvY
    %idx_map2:记录D2中投影点(都有unique_corr)在pc1中的对应corr
    %nonunique_nonproj:indice of occluded points
    xyz_pc1_nonunique = select(pc1,not_unique_corr(:,1));
%     y_pc1 = xyz_pc1_nonunique.Color(:,1); 
    xyzidx_pc1 = [xyz_pc1_nonunique.Location(:,:), not_unique_corr(:,1)];
    [D2, idx_map2, nonunique_nonproj] = zbuffer_forward_proj(xyzidx_pc1,camera_para,pc1);
%     [~, D2, ~, idx_map2]= transformXYZ2UVDI(xyzidx_pc1, camera_para, y_pc1, unique_corr);
   
%    transformationMap = tripple_guided_JBF_v3(D1, D3, Y1, Y3, mask, transformationMap1, transformationMap2);

%  Mention:here num of non_inf points in transformationMap2 is less than that in idx_map2 
%idx_need_to_be_added_to_nonunique_nonproj:record indice of points that don't have nearest neighbor unique_corr points
   [transformationMap2, idx_need_to_be_added_to_nonunique_nonproj]= transformation_JBF(D1,D2,transformationMap1, idx_map2);
   
    
   nonunique_nonproj = [nonunique_nonproj;idx_need_to_be_added_to_nonunique_nonproj];
   nonunique_nonproj = sort(nonunique_nonproj);
   
   %%===get warped point from transformationMap(processing points belonging to status 1 and 2)===
   warped_pc_have_trans1 = warp(pc1, idx_map1, transformationMap1);
   warped_pc_have_trans2 = warp(pc1, idx_map2, transformationMap2);
   
%    warped_pc_use_top_node = warp_use_top_node();
   pc_use_top_node = select(pc1,nonunique_nonproj);
   warped_pc_use_top_node = pctransform(pc_use_top_node,Tmat{1}{1});
   
   %%combine all the warped pts
   warpedPointcloud = pcmerge(warped_pc_have_trans1,warped_pc_have_trans2,3); %3mm
   warpedPointcloud = pcmerge(warpedPointcloud,warped_pc_use_top_node,3);
   array = single(warpedPointcloud.Location(:,:));
   warpedPointcloud = pointCloud(array);
   
   
   warpedPointcloud.Color = repmat(uint8([255,0,0]),warpedPointcloud.Count,1);
   pc2.Color = repmat(uint8([0,0,255]),pc2.Count,1);
   figure(12),pcshow(warpedPointcloud);hold on; pcshow(pc2);hold off;
   title('warpedPointcloud(R) and pc2(B)');drawnow;
   
%    pcwrite(warpedPointcloud,... 
%         ['./node_seg/output/pointcloud/pc_','1','.pcd'],...
%         'Encoding','ascii');
end


function warped_pc = warp(pc, idx_map, transformationMap)
    [H, W] = size(idx_map); cnt = 0;
    warped_pc = zeros(H*W,3);
    for r = 1:H
        for c = 1:W
            if idx_map(r,c) ~= 0 && transformationMap(r,c,1) ~= inf
                cnt = cnt + 1;
                rodri = transformationMap(r,c,:); 
                rodri = reshape(rodri, [1,6]);
                R = rodrigues(rodri(1,1:3)); t = rodri(1,4:6);
                pt = pc.Location(idx_map(r,c),:);
                warped_pt = R * pt' + t';
                warped_pc(cnt,:) = warped_pt';
            end
        end
    end
    warped_pc = warped_pc(1:cnt,:);
    warped_pc = pointCloud(warped_pc);
end


function [D, idx_map, nonunique_nonproj] = zbuffer_forward_proj(xyzidx_pc,camera_para, pc_total)
    D = ones(480,640)*inf; idx_map = zeros(480,640);
    nonunique_nonproj = zeros(size(xyzidx_pc,1),1); 
    cnt = 0; %count the nonunique_nonproj points
    for i = 1:size(xyzidx_pc,1)
        x = xyzidx_pc(i,1); y = xyzidx_pc(i,2); z = xyzidx_pc(i,3);
        u = round(x/z*camera_para.fx + camera_para.cx + 0.5);
        v = round(y/z*camera_para.fy + camera_para.cy + 0.5);
        d = z;
        
        if D(v,u) > d % the i_th point is not occluded
            D(v,u) = d;
            if idx_map(v,u) ~= 0
                cnt = cnt + 1;
                nonunique_nonproj(cnt,1) = idx_map(v,u); 
                idx_map(v,u) = xyzidx_pc(i,4);
            else 
                idx_map(v,u) = xyzidx_pc(i,4);
            end
        else % the i_th point is occluded
            cnt = cnt + 1;
            nonunique_nonproj(cnt,1) = xyzidx_pc(i,4);
        end
    end
    nonunique_nonproj = nonunique_nonproj(1:cnt,1);
    nonunique_nonproj = sort(nonunique_nonproj);
    
    %%check visualization
%     figure(1),imshow(D,[]);
%     pc_non = select(pc_total,nonunique_nonproj);
%     pc_non.Color = repmat(uint8([255,0,0]),pc_non.Count,1);
%     pc = pointCloud(xyzidx_pc(:,1:3));
%     pc.Color = repmat(uint8([0,255,0]),pc.Count,1);
%     figure(2),pcshow(pc);hold on; pcshow(pc_non);hold off;
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


function transformationMap = idx_to_transformation(idx_map, pc_bestNode_distr, Tmat)
    %===先把Tmat{4}转成[α, β，γ，t1,t2,t3]
    Tmat_6DoF = zeros(size(Tmat{4},2),1);
    for i = 1:size(Tmat_6DoF)
        T = Tmat{4}{i}.T';
        R = T(1:3,1:3); t = T(1:3,4);
        Tmat_6DoF(i,1:3) = rodrigues(R);
        Tmat_6DoF(i,4:6) = t;
    end
    transformationMap = ones(480,640,6)*inf;  %[α,β，γ，t1,t2,t3]
    for r = 1:480
        for c = 1:640
            idx = idx_map(r,c);
            if idx==0, continue; end
            t_idx = pc_bestNode_distr(idx);
            transformationMap(r,c,:) = Tmat_6DoF(t_idx,:);
        end
    end
end
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          




