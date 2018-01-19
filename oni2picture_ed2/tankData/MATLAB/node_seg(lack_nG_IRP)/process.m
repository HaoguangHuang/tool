%% WARPED_PC
% pc1:pointCloud. Here pc1 is in last camera frame coo
% pc2:pointCloud. back-projected from live depth map
%
% nodeGraph:{frame_no, data}------------------------------------------------------>
%                      data:{data_1, data_2, ..., data_layers}-------------------->
%                                                 data_L:{Tmat, node_position}---->
%                                                         Tmat: n*1 affine3d
%                                                         node_position: n*3 xyz

function [warpedPointcloud, nodeGraph]= process(pc1, pc2, para_set, nodeGraph, frame_no, cnt)
    global debug_mode;
    layers = para_set.nodeGraph_layers;
    camera_para = para_set.camera_para;
    node_r = para_set.node_radius;
    %======grid filter======
%     pc1 = pcdownsample(pc1, 'gridAverage', 3); % grid filter. r = 3mm
%     pc1 = pcdenoise(pc1);
    
    %% get node_set_live
    % use nodeGraph to transform nodes from 'frame_no-1' coo into 'frame_no' coo
    node_set_updated = update_node(nodeGraph, cnt);
    
    %=============visualize result of node segmentation=============%
    if debug_mode, drawNodeSeg(pc1,node_r,node_set_updated); drawNodeSeg(pc1,node_r*1.25,node_set_updated); end
    
    %% create node tree
    node_tree = createNodeTree(node_set_updated,para_set.nodeGraph_layers);
    
    %% find pointcloud belongs to each node
    node_r_pc1 = para_set.node_radius*1.25; node_r_pc2 = para_set.node_radius*1.25*1.1;
    [pc_set1, pc_set1_node_index]= distr_pc(pc1,node_r_pc1,node_set_updated,layers,1);  %distribute pointcloud
    [pc_set2, ~]= distr_pc(pc2,node_r_pc2,node_set_updated,layers,2);
    
    
    %% hierarchical node ICP
    [Tmat, rmse]= hierarchical_ICP(pc_set1,pc_set2,para_set.nodeGraph_layers, node_tree);
    
    %% find best connection between node and each point in pointcloud
    thres_outlier = 2;        %mm
    [outlier_index, pc_bestNode_distr] = findBestNode(pc1,pc2,Tmat, pc_set1_node_index, layers, thres_outlier);
    corrIndex = find_unique_corr(pc_bestNode_distr, pc1, pc2, Tmat{layers}, thres_outlier);
    
        %======visualize pc1 in coordinate of pc2 with unique correspondence transformation======
    if debug_mode, visualize_with_corr(pc1, pc2, corrIndex, Tmat{layers},pc_bestNode_distr); end
    
        %======visualize the optical flow map, and find whether the unique_correspondences are true or not======
    if debug_mode, visualize_energy_map(pc1, pc2, corrIndex, camera_para); end
    
    %% sparse2dense_v2,interpolate sparse unique correspondence point cloud into dense point cloud======
    warpedPointcloud = get_warped_pointcloud(pc1, pc2, corrIndex, camera_para, Tmat, pc_bestNode_distr);

    %% construct a nodeGraph======
    nodeG_thisFrame = cell(1,layers);
    for i = 1:layers
        nodeG_thisFrame{i} = {Tmat{i}',node_set_updated{i}.Location(:,:)};
    end
    nodeGraph(cnt,:) = {frame_no, nodeG_thisFrame};
end



function node_set_updated = update_node(nodeGraph, cnt)
    data = nodeGraph{cnt-1,2}; 
    layers = size(data,2);
    node_set_updated = cell(1,layers);
    for L = 1:layers
        node_num = size(data{L}{2},1);
        Tmat = data{L}{1}; pos = data{L}{2};
        un_array = zeros(node_num,3); %updated nodes array
        for n = 1:node_num
            un = transformPointsForward(Tmat{n},pos(n,:));
            un_array(n,:) = un;
        end
        node_set_updated{L} = pointCloud(un_array);
    end
    
end


function drawNodeSeg(pc,n_r,node_set)
    [x,y,z] = sphere(15);    
    for L = 1:length(n_r)
        figure(L); pcshow(pc);hold on;
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


%% distrPc: distribute pointcloud to nodes in each layers
%        pc_set: cell. Record pointcloud in nodes of each layers
% pc_node_index: pc.Count*node_set{4}.Count array. Record the relation
%                between pointcloud and nodes in layer 4. If point 1 
%                and node 4 connect, then pc_node_index(1,4)=1; 
function [pc_set, pc_node_index]= distr_pc(pc,leaf_size,node_set,layers,flag)
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
                [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPoint','Verbose',true);
                disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
            else % L=2,3,4
                if moving_pc{L}{n}.Count < 3 || fixed_pc{L}{n}.Count < 3
                    Tmat{L}{n} = Tmat{L-1}{node_tree{L}(n)};
                    rmse{L}{n} = inf;
                    disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
                    continue;
                end
                [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPoint','Verbose',true,...
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