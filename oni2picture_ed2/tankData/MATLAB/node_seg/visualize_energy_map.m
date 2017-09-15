%% VISUALIZE_ENERGY_MAP:visualize the unique correspondence via constructing an energy map
%                     pc1:pointCloud.     moving point
%                     pc2:pointCloud.     fixing point
%             unique_corr:array.          Its size is pc1.Count*3. [indice, correspondence, dist]
%             camera_para:struct.         containing fx, fy, cx, cy
function visualize_energy_map(pc1, pc2, unique_corr, camera_para)
    if nargin<1, load('pc.mat','pc_197'); pc1 = pc_197; clear pc_197; end
    if nargin<2, load('pc.mat','pc_198'); pc2 = pc_198; clear pc_198; end
    if nargin<3, load('corrIndex.mat','corrIndex_wc03_thres2'); 
        unique_corr = corrIndex_wc03_thres2;  clear corrIndex_wc03_thres2; 
    end
    if nargin<4, camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202); end
    if nargin<5, load('relation_pc_dmap_backUp.mat','relation_pc1_dmap'); end
    
    pixelRadius = 20;  %pixel
    depthRadius = 10;   %mm
    %%========find (u,v,d) of correspondence pair in depth map========
    validIndex = unique_corr(:,2)>0;  %exclude invalid correspondence
    unique_corr = unique_corr(validIndex,:);  
    xyz_pc1 = pc1.Location(validIndex,:); xyz_pc2 = pc2.Location(validIndex,:);
    uvd_pc1 = transformXYZ2UVD(xyz_pc1, camera_para);
    uvd_pc2 = transformXYZ2UVD(xyz_pc2, camera_para);
    %%========construct energy map========
    delta_u = uvd_pc1(:,1) - uvd_pc2(:,1);  
    delta_v = uvd_pc1(:,2) - uvd_pc2(:,2);
    delta_d = uvd_pc1(:,3) - uvd_pc2(:,3);
    
    delta_u(delta_u>pixelRadius) = pixelRadius;  delta_u(delta_u<-pixelRadius) = -pixelRadius;
    delta_v(delta_v>pixelRadius) = pixelRadius;  delta_v(delta_v<-pixelRadius) = -pixelRadius;
    delta_d(delta_d>depthRadius) = depthRadius;  delta_d(delta_d<-depthRadius) = -depthRadius;
    
    norm_delta_u = mapminmax(delta_u',0,255);  %R
    norm_delta_v = mapminmax(delta_v',0,255);  %G
    norm_delta_d = mapminmax(delta_d',0,255);  %B
    
    energyMap = zeros(480,640,3);
    for i = 1:size(unique_corr,1)
        energyMap(uvd_pc1(i,1), uvd_pc1(i,2), 1) = norm_delta_u(i);
        energyMap(uvd_pc1(i,1), uvd_pc1(i,2), 2) = norm_delta_v(i);
        energyMap(uvd_pc1(i,1), uvd_pc1(i,2), 3) = norm_delta_d(i);
    end
    
    figure(1),imshow(uint8(energyMap)),title('energy map');
end


function uvd_pc = transformXYZ2UVD(xyz_pc, camera_para)
    uvd_pc = zeros(size(xyz_pc));
    uvd_pc(:,3) = xyz_pc(:,3);                                                       %d = z
    for i = 1:size(xyz_pc,1)
        uvd_pc(i,1) = round(xyz_pc(i,1)*camera_para.fx/xyz_pc(i,3)+camera_para.cx);  %u = (x*f_x)/z + c_x
        uvd_pc(i,2) = round(xyz_pc(i,2)*camera_para.fy/xyz_pc(i,3)+camera_para.cy);  %v = (y*f_y)/z + c_y
    end
    
end




