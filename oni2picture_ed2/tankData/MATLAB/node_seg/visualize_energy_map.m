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
%     if nargin<5, load('relation_pc_dmap_backUp.mat','relation_pc1_dmap'); end
    
    pixelRadius = 2;  %pixel
    depthRadius = 6;  %mm
    %%========find (u,v,d) of correspondence pair in depth map========
    validIndex = unique_corr(:,2)>0;  %exclude invalid correspondence
    unique_corr = unique_corr(validIndex,:);  
    xyz_pc1 = select(pc1, unique_corr(:,1));  xyz_pc1 = xyz_pc1.Location(:,:);
    xyz_pc2 = select(pc2, unique_corr(:,2));  xyz_pc2 = xyz_pc2.Location(:,:);
    
    vud_pc1 = transformXYZ2UVD(xyz_pc1, camera_para);
    vud_pc2 = transformXYZ2UVD(xyz_pc2, camera_para);
    %%========construct energy map========
    delta_v = vud_pc1(:,1) - vud_pc2(:,1);  %%%%%%%%%%%%%
    delta_u = vud_pc1(:,2) - vud_pc2(:,2);
    delta_d = vud_pc1(:,3) - vud_pc2(:,3);
    
%     getEnergyMap_all_without_direction(delta_u, delta_v, delta_d, vud_pc1, unique_corr, pixelRadius, depthRadius);
%     getEnergyMap_with_direction(delta_u, vud_pc1, unique_corr, pixelRadius, 'delta\_u');
%     getEnergyMap_with_direction(delta_v, vud_pc1, unique_corr, pixelRadius, 'delta\_v');
%     getEnergyMap_with_direction(delta_d, vud_pc1, unique_corr, depthRadius, 'delta\_d');
    get_optical_flow_map_with_uv(delta_u, delta_v, vud_pc1, unique_corr, pixelRadius);
end


function vud_pc = transformXYZ2UVD(xyz_pc, camera_para)
    vud_pc = zeros(size(xyz_pc));
    vud_pc(:,3) = xyz_pc(:,3);                                                       %d = z
    for i = 1:size(xyz_pc,1)
        vud_pc(i,2) = round(xyz_pc(i,1)*camera_para.fx/xyz_pc(i,3)+camera_para.cx);  %u = (x*f_x)/z + c_x
        vud_pc(i,1) = round(xyz_pc(i,2)*camera_para.fy/xyz_pc(i,3)+camera_para.cy);  %v = (y*f_y)/z + c_y
    end
end


%% GET_OPTICAL_FLOW_MAP_WITH_UV
%   Here optical flow map can represent the degree and direction of delta_u and delta_v
function get_optical_flow_map_with_uv(delta_u, delta_v, vud_pc1, unique_corr, pixelRadius)
    delta_u(delta_u>pixelRadius) = pixelRadius;  delta_u(delta_u<-pixelRadius) = -pixelRadius;
    delta_v(delta_v>pixelRadius) = pixelRadius;  delta_v(delta_v<-pixelRadius) = -pixelRadius;    
    
     %======可视化reference=======
    indice_hang = linspace(-pixelRadius, pixelRadius, 2*pixelRadius+1);  %行向量。用于构造reference球
    indice_lie = linspace(pixelRadius, -pixelRadius, 2*pixelRadius+1);  %列向量
    indice_hang = mapminmax(indice_hang, 0, 255); indice_lie = mapminmax(indice_lie, 0, 255);
    ref = zeros(2*pixelRadius+1, 2*pixelRadius+1, 3);  %reference board。3 channels are [u,v,0]
    ref(:,:,1) = repmat(indice_hang, 2*pixelRadius+1, 1);
    ref(:,:,2) = repmat(indice_lie', 1, 2*pixelRadius+1);
    figure(111),imshow(uint8(ref)),title('reference board');
    
    %======可视化optical flow map======
    %绿色是v垂直方向，红色是u水平方向
    opticalFlowMap = zeros(480,640,3);
    for i = 1:size(unique_corr,1)
        if vud_pc1(i,1)<1 || vud_pc1(i,1)>480 || vud_pc1(i,2)< 1 || vud_pc1(i,2)>640, continue; end
        %下面的delta_v加负号，是为了确保delta_v<0时，点能出现在第三或第四象限
        r = ref(-delta_v(i)+pixelRadius+1,delta_u(i)+pixelRadius+1,1);  %delta需要偏移pixelRaidus+1个单位才能对准reference board
        g = ref(-delta_v(i)+pixelRadius+1,delta_u(i)+pixelRadius+1,2);
        opticalFlowMap(vud_pc1(i,1), vud_pc1(i,2), 1) = r; 
        opticalFlowMap(vud_pc1(i,1), vud_pc1(i,2), 2) = g;
    end
    %%======change background color to white======
    mask_bg = opticalFlowMap(:,:,1) == 0 & opticalFlowMap(:,:,2) == 0 & opticalFlowMap(:,:,3) == 0; 
    R = mask_bg * 255; G = mask_bg * 255; B = mask_bg * 255;
    opticalFlowMap(:,:,1) = R + opticalFlowMap(:,:,1);
    opticalFlowMap(:,:,2) = G + opticalFlowMap(:,:,2);
    opticalFlowMap(:,:,3) = B + opticalFlowMap(:,:,3);
    
    figure(1),imshow(uint8(opticalFlowMap)),title('optical flow map');
end


%% GET_ENERGY_MAP_ALL_WITHOUT_DIRECTION
%   Here energyMap only can represent the degree of deviation, but cannot represent 
% the direction of deviation.
function getEnergyMap_all_without_direction(delta_u, delta_v, delta_d, vud_pc1, unique_corr, pixelRadius, depthRadius)
    delta_u(delta_u>pixelRadius) = pixelRadius;  delta_u(delta_u<-pixelRadius) = -pixelRadius;
    delta_v(delta_v>pixelRadius) = pixelRadius;  delta_v(delta_v<-pixelRadius) = -pixelRadius;
    delta_d(delta_d>depthRadius) = depthRadius;  delta_d(delta_d<-depthRadius) = -depthRadius;
    
    delta_u(delta_u<0) = abs(delta_u(delta_u<0));
    delta_v(delta_v<0) = abs(delta_v(delta_v<0));
    delta_d(delta_d<0) = abs(delta_d(delta_d<0));
    
    norm_delta_u = mapminmax(delta_u',0,255);  %R
    norm_delta_v = mapminmax(delta_v',0,255);  %G
    norm_delta_d = mapminmax(delta_d',0,255);  %B
    
    energyMap = zeros(480,640,3); 
    for i = 1:size(unique_corr,1)
        if vud_pc1(i,1)<1 || vud_pc1(i,1)>480 || vud_pc1(i,2)< 1 || vud_pc1(i,2)>640, continue; end
        energyMap(vud_pc1(i,1), vud_pc1(i,2), 1) = norm_delta_u(i);
        energyMap(vud_pc1(i,1), vud_pc1(i,2), 2) = norm_delta_v(i);
        energyMap(vud_pc1(i,1), vud_pc1(i,2), 3) = norm_delta_d(i);
    end
    %%====change background color to white====
    %   Here energyMap only can represent the degree of deviation, but cannot represent 
    %     the direction of deviation.
    mask_bg = energyMap(:,:,1) == 0 & energyMap(:,:,2) == 0 & energyMap(:,:,3) == 0; 
    R = mask_bg * 255; G = mask_bg * 255; B = mask_bg * 255;
    energyMap(:,:,1) = R + energyMap(:,:,1);
    energyMap(:,:,2) = G + energyMap(:,:,2);
    energyMap(:,:,3) = B + energyMap(:,:,3);
    
    figure(1),imshow(uint8(energyMap)),title('energy map');
end

%% GET_ENERGY_MAP_WITH_DIRECTION
%   This function visualize scene flow map via a component(u or v or d), with direction
%      choice : string. Its value is 'delta_u' or 'delta_v' or 'delta_d'
%        Note : R represent delta is minus, G represent delta is zero, and B represent positive 
function getEnergyMap_with_direction(delta, vud_pc1, unique_corr, Radius, choice)
    delta(delta>Radius) = Radius;  delta(delta<-Radius) = -Radius;
    n = size(unique_corr,1);
    minus_R(:,1) = linspace(1,n,n);
    zeros_G = minus_R;     positive_B  = minus_R; 
    
    minus_R = minus_R(delta < 0);
    zeros_G = zeros_G(delta == 0); 
    positive_B = positive_B(delta > 0);
    
    energyMap = zeros(480,640,3);  
    %%====construct R component via minus delta====
    for i = 1:size(minus_R,1)
        u = vud_pc1(minus_R(i),1); v = vud_pc1(minus_R(i),2);
        energyMap(u, v, 1) = 255;
    end
    %%====construct G component via minus delta====
    for i = 1:size(zeros_G,1)
       u = vud_pc1(zeros_G(i),1); v = vud_pc1(zeros_G(i),2);
        energyMap(u, v, 2) = 255;
    end
    %%====construct B component via minus delta====
    for i = 1:size(positive_B,1)
        u = vud_pc1(positive_B(i),1); v = vud_pc1(positive_B(i),2);
        energyMap(u, v, 3) = 255;
    end
    
     %%====change background color to white====
    %   Here energyMap only can represent the degree of deviation, but cannot represent 
    %     the direction of deviation.
    mask_bg = energyMap(:,:,1) == 0 & energyMap(:,:,2) == 0 & energyMap(:,:,3) == 0; 
    R = mask_bg * 255; G = mask_bg * 255; B = mask_bg * 255;
    energyMap(:,:,1) = R + energyMap(:,:,1);
    energyMap(:,:,2) = G + energyMap(:,:,2);
    energyMap(:,:,3) = B + energyMap(:,:,3);
    
    figure,imshow(uint8(energyMap)),title([choice, ' with direction']);
end











