%% REG_MAIN: Registration Main Function 
function reg_main
    close all;  global debug_mode; debug_mode = 1;
    if nargin < 1, frameNum = 2; end %197-198
    camera_para = struct('fx',504.261,'fy',503.905,'cx',352.457,'cy',272.202);
%% read depth data and tranform to pointcloud
    pc = cell(1,frameNum); D = cell(1,frameNum); Yuv = cell(1,frameNum);
    for i = 1:frameNum
        D{i} = imread(['E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\d_',int2str(i+196),'.png']); 
        Yuv{i} = imread(['E:\dataSet\Wajueji_2\processedData\extractdata_afterDRev\c_',int2str(i+196),'.png']);
        eval(['[pc{i}, relation_pc',int2str(i),'_dmap]= transformUVD2XYZInten(double(D{i}), Yuv{i}(:,:,1), camera_para);']);
        pc{i} = pcdenoise(pc{i});
    end
    roi = [-230,inf;-inf,inf;0,980]; %197-198
%     roi = [-210,240;-inf,inf;0,960]; %198-199,199-200
    if debug_mode, figure(101),pcshow(pc{1}), title('before ROI filter'); end
    pc{1} = select(pc{1},findPointsInROI(pc{1},roi));%place wajueji in ROI
    if debug_mode, figure(100),pcshow(pc{1}),title('after ROI filter'); end
    
%     relation_pc1_dmap = find_relation(relation_pc1_dmap, pc{1}); %this function is a rubbish
%     save('relation_pc_dmap.mat','relation_pc1_dmap');
%% hierarchical node
    layers = 4; 
    node_r = [500,250,200,150]/2; %mm. orient to node num = 1,4,8,12
    node_set = cell(1,layers);    %save the node position in every layers 
    for L = 1:layers
        node_set{L} = pcdownsample(pc{1}, 'gridAverage', node_r(L)*2);  %grid size should be 2*r
    end
    %%=============visualize result of node segmentation=============%%
%     drawNodeSeg(pc,node_r,node_set); drawNodeSeg(pc,node_r*1.25,node_set);
    
%% create hierarchical node tree
    node_tree = createNodeTree(node_set,layers);
    
%% find pointcloud belongs to each node
    node_r_pc1 = node_r*1.25; node_r_pc2 = node_r*1.25*1.1;
    [pc_set1, pc_set1_node_index]= distrPc(pc{1},node_r_pc1,node_set,layers,1);  %distribute pointcloud
    [pc_set2, ~]= distrPc(pc{2},node_r_pc2,node_set,layers,2);
    
%% node ICP
    [Tmat, rmse]= hierarchical_ICP(pc_set1,pc_set2,layers, node_tree);
    this_file = 'E:\Code\vs2010\oni2picture_ed2\oni2picture_ed2\tankData\MATLAB\node_seg\';
    if 0 %debug_mode 
    save([this_file,'Tmat_wc03.mat'],'Tmat');  save([this_file,'rmse_wc03.mat'],'rmse');
    analyseTmat(Tmat);%visualize rotation angle and translation
    end
    
%% find best connection between node and each point in pc
    thres_outlier = 2;        %mm
    thres_corr = 2;           %mm
    [outlier_index, pc_bestNode_distr] = findBestNode(pc{1},pc{2},Tmat, pc_set1_node_index, layers, thres_outlier);
    corrIndex = find_unique_corr(pc_bestNode_distr, pc{1}, pc{2}, Tmat{layers}, thres_corr);
%% visualize pc1 in coordinate of pc2 with unique correspondence transformation
    if debug_mode, visualize_with_corr(pc{1}, pc{2}, corrIndex, Tmat{layers},pc_bestNode_distr); end
%% 可视化optical flow map，观察unique_correspondence找得对不对    
    visualize_energy_map(pc{1}, pc{2}, corrIndex, camera_para);
    
%% sparse2dense_v2,把sparse unique correspondence point cloud插值成dense point cloud
    [denseMap2, idx_map2]= sparse2dense_v2(pc{1}, pc{2}, corrIndex, camera_para, Tmat, pc_bestNode_distr);
%     eval(['save(']);
end

 
 
 
 
 %%===============================================================================================================
 %%===============================================================================================================
% relation_pc_dmap:array. [u,v,x,y, cloud_indice]
function [p, relation_pc_dmap]= transformUVD2XYZInten(d, Intensity, c_pa)
    [H, W] = size(d);
    p_array(:,3) = reshape(d,H*W,1);        %z can obtain from var d
    y_array = zeros(H*W,1); relation_pc_dmap = zeros(H*W, 5);
    for u = 1:W
        for v = 1:H
            p_array((u-1)*H+v,1) = (u - c_pa.cx) * d(v,u)/ c_pa.fx;
            p_array((u-1)*H+v,2) = (v - c_pa.cy) * d(v,u)/ c_pa.fy;
            y_array((u-1)*H+v,1) = Intensity(v,u);% y_array:n*1, one dimension is y intensity
            relation_pc_dmap((u-1)*H+v,1:4) = [u, v, p_array((u-1)*H+v,1), p_array((u-1)*H+v,2)];
        end
    end
    index = p_array(:,3)>0;
    
    relation_pc_dmap = relation_pc_dmap(index,:);
    relation_pc_dmap(:,5) = linspace(1, size(relation_pc_dmap,1), size(relation_pc_dmap,1));
    p_array = p_array(index,:); 
    p = pointCloud(p_array);
    p.Color = [y_array(index,:),uint8(zeros(p.Count,2))];   % give ycbcr intensity to pc
end

function drawNodeSeg(pc,n_r,node_set)
    [x,y,z] = sphere(15);    
    for L = 1:length(n_r)
        figure(L); pcshow(pc{1});hold on;
        plot3(node_set{L}.Location(:,1),node_set{L}.Location(:,2),node_set{L}.Location(:,3),'r.','markerSize',20);
        for c = 1:node_set{L}.Count
        mesh(x*n_r(L)+node_set{L}.Location(c,1),...
        y*n_r(L)+node_set{L}.Location(c,2),...
        z*n_r(L)+node_set{L}.Location(c,3));
        alpha(0);
        end
        xlabel('X'),ylabel('Y'),zlabel('Z'); title(['layer ',int2str(L)]);
        hold off;
    end
end

%%     distrPc: distribute pointcloud to nodes in each layers
%        pc_set: cell. Record pointcloud in nodes of each layers
% pc_node_index: pc.Count*node_set{4}.Count array. Record the relation
%                between pointcloud and nodes in layer 4. If point 1 
%                and node 4 connect, then pc_node_index(1,4)=1; 
function [pc_set, pc_node_index]= distrPc(pc,leaf_size,node_set,layers,flag)
    global debug_mode;
    pc_set = cell(1,layers);
%     pc.Color = repmat(uint8([0,255,0]),pc.Count,1);
    pc_node_index = zeros(pc.Count, node_set{layers}.Count);
    for L = 1:layers
        pc_set{L} = cell(1,node_set{L}.Count);
        for n = 1:node_set{L}.Count
            [indice, ~] = findNeighborsInRadius(pc,node_set{L}.Location(n,:),leaf_size(L));
            pc_set{L}{n} = select(pc,indice);% pointcloud within radius
            if L == layers
                for i = 1:size(indice,1)         % distribute node index to pointcloud
                    pc_node_index(indice(i),n) = 1;
                end
            end
            %%===============visualize result===================%%
            p_r = select(pc,indice);
            p_r.Color = repmat(uint8([255,0,0]),p_r.Count,1);
            if debug_mode
            figure(10+flag);subplot(3,4,n);pcshow(pc),hold on;pcshow(p_r);title(['layer=',int2str(L),',the ',int2str(n),'th node']);
            drawnow;
            end
        end
        hold off;
    end
     %%===============visualize pointcloud controlled by more than 2 nodes===================%%
     pc_node_morethan2_index = zeros(pc.Count,1);
     for i = 1:pc.Count
        pc_node_morethan2_index(i,1) = i;
     end
     pc_node_morethan2_index = pc_node_morethan2_index(sum(pc_node_index,2)>1);
     pc_node_morethan2 = select(pc,pc_node_morethan2_index);
     pc_node_morethan2.Color = repmat(uint8([255,0,0]),pc_node_morethan2.Count,1);
     if debug_mode
     figure(33),pcshow(pc);hold on; pcshow(pc_node_morethan2);title('pointcloud controlled by more than one node');hold off;
     drawnow; 
     end
end

function [Tmat, rmse] = hierarchical_ICP(moving_pc,fixed_pc,layers, node_tree)
    Tmat = cell(1,layers); rmse = cell(1,layers);
    for L = 1:layers
        Tmat{L} = cell(1,size(moving_pc{L},2)); rmse{L} = cell(1,size(moving_pc{L},2));
        for n = 1:size(moving_pc{L},2)
            if L == 1 
                [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid_intensity(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPoint','Verbose',true);
                disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
            else % L=2,3,4
                [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid_intensity(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPoint','Verbose',true,...
                    'InitialTransform',Tmat{L-1}{node_tree{L}(n)});
                disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
            end
        end
    end
end

%% createNodeTree:every element in node tree record the connection with the parent node( in upper class)
function node_tree = createNodeTree(node_set,layers)
    if layers < 2
        error('need not create node tree');
    end
    node_tree = cell(1,layers); node_tree{1}{1} = 0;
    for L = 2:layers
        node_tree{L} = zeros(1,node_set{L}.Count);
        for i = 1:node_set{L}.Count
            node_tree{L}(i) = findConnection(node_set{L}.Location(i,:),node_set{L-1}.Location);
        end
    end
end

%% findConnection:find connection between son node and parent node
%       son: 1*3 array. position of son node
% parent_pc: n*3 array. parent node  
%      edge: scalar. a guidance from son node to parent node
function edge = findConnection(son,parent)
    son_mat = repmat(son,size(parent,1),1);
    dist = (parent - son_mat).^2;
    [~, edge] = min(sum(dist,2)); % need not to sqrt
end

%% FIND_RELATION: 
%  Since the pointcloud indice will be in chaos after findPointsInROI, here find the correct relation between
%    pc1 and depth map.
function newRelation = find_relation(relation_pc1_dmap, pc1)
    newRelation = zeros(pc1.Count, 5);
    parpool('local',4);
    parfor i = 1:pc1.Count
        for j = 1:size(relation_pc1_dmap,1)
            if pc1.Location(i,1) == relation_pc1_dmap(j,3) && pc1.Location(i,2) == relation_pc1_dmap(j,4)
                newRelation(i,:) = [relation_pc1_dmap(j,1:4), i];
            end
        end
        disp([int2str(i),' finished!']);
    end
    delete(gcp('nocreate'));
end


