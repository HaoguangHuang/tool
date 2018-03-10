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
    windowSize = para_set.windowSize;
    radius_coff = para_set.radius_coff;
    %% get node_set_live
    % use nodeGraph to transform nodes from 'frame_no-2' coo into 'frame_no-1' coo
%     node_set_updated = update_node(nodeGraph, cnt);
    tic;
    node_set_updated = update_node_mean_window(nodeGraph, cnt, windowSize, frame_no);
    fprintf('update_node_mean_window time = %d\n',toc);

    for L = 1:layers
        node_set_updated{1,L} = attach_new_node_to_NNP(node_set_updated{1,L},pc1);
    end
    
    %=============visualize result of node segmentation=============%
    if debug_mode, drawNodeSeg(pc1,node_r,node_set_updated); drawNodeSeg(pc1,node_r*radius_coff,node_set_updated); end
    
    
    %% find pointcloud belongs to each node
    node_r_pc1 = para_set.node_radius*radius_coff; node_r_pc2 = para_set.node_radius*radius_coff*1.1;
    tic;
    [pc_set1, pc_set1_node_index]= distr_pc(pc1,node_r_pc1,node_set_updated,layers,1);  %distribute pointcloud
    fprintf('distr_pc time = %d\n',toc);
    %% nodeGraph module 
    %======check how many pts don't belong to any node in the highest layer======
    if 1
        record = sum(pc_set1_node_index, 2);
        pt_belongTo_noNode_idx = record == 0;
        NEED_TO_ADD_HIGHEST_NEW_NODE = sum(pt_belongTo_noNode_idx) > para_set.OOR_thres;
        if  NEED_TO_ADD_HIGHEST_NEW_NODE %need to add new nodes in the highest layer
            indice = 1:pc1.Count;
            pc_belongTo_noNode = select(pc1,indice(pt_belongTo_noNode_idx));
            if debug_mode
                pc_belongTo_noNode.Color = repmat(uint8([255,0,0]),pc_belongTo_noNode.Count,1);
                figure(77),pcshow(pc1),hold on;
                pcshow(pc_belongTo_noNode);
                title(sprintf('pc_belongTo_noNode,pts=%d',pc_belongTo_noNode.Count)); hold off;
            end
            node_added_set = pcdownsample(pc_belongTo_noNode, 'gridAverage', node_r(layers)*2);
            if debug_mode
                figure(34),pcshow(pc1),hold on;
                plot3(node_added_set.Location(:,1),node_added_set.Location(:,2),node_added_set.Location(:,3),...
                    'ro','Markersize',10);
                hold off;
            end
            %======change position of new added nodes======
            node_added_set = attach_new_node_to_NNP(node_added_set, pc1);
            
            %======exclude some redundant nodes======
            node_added_set = excludeAddedNodes(node_added_set, pc1, para_set);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            node_set_updated = add_new_node_to_highest_layer(node_set_updated, node_added_set, layers);
            
            %===check whether each nodes can be conquered by any of the node from higher layer===
%             [HAVE_ADD_LOWER_LAYER_NODE, node_set_updated]= update_node_set(node_set_updated, node_r);
            [HAVE_ADD_LOWER_LAYER_NODE, node_set_updated]= update_node_set_mean_window(node_set_updated, node_r,pc1);
            
            if HAVE_ADD_LOWER_LAYER_NODE, disp('-------Have updated node_set!-------');  end

            %======visualize points that don't belong to any node
            if debug_mode
                pc1.Color = repmat(uint8([255,0,0]),pc1.Count,1);
                pc_belongTo_noNode.Color = repmat(uint8([0,255,0]),pc_belongTo_noNode.Count,1);
                figure(12),pcshow(pc1),hold on, pcshow(pc_belongTo_noNode),hold off, title('pc1(R) and pc\_belongTo\_noNode(G)');
            end

            [pc_set1, pc_set1_node_index]= distr_pc(pc1,node_r_pc1,node_set_updated,layers,1);  %redistribute pointcloud
        end
    end
    
    [pc_set2, ~]= distr_pc(pc2,node_r_pc2,node_set_updated,layers,2);
    
    
    %% create node tree
    node_tree = createNodeTree(node_set_updated, layers);
    
    %% hierarchical node ICP
    tic;
    [Tmat, rmse]= hierarchical_ICP(pc_set1,pc_set2,para_set.nodeGraph_layers, node_tree);
    fprintf('hierarchical_ICP time = %d\n',toc);
    %% find best connection between node and each point in pointcloud
    thres_outlier = 2;        %mm
    tic;
%     [outlier_index, pc_bestNode_distr] = findBestNode(pc1,pc2,Tmat, pc_set1_node_index, layers, thres_outlier);
    [outlier_index, pc_bestNode_distr] = findBestNode2(pc1,pc2,Tmat,pc_set1_node_index, layers, thres_outlier, node_set_updated);
    fprintf('findBestNode time = %d\n',toc);
    
    tic;
    corrIndex = find_unique_corr(pc_bestNode_distr, pc1, pc2, Tmat{layers}, thres_outlier);
    fprintf('find_unique_corr time = %d\n',toc);
%     uniq_name = ['corrIndex_',int2str(frame_no),'_',int2str(frame_no+1)];
%     eval([uniq_name,'=corrIndex']);
%     save('./output/result/corr_1_200.mat',uniq_name,'-append');
    
        %======visualize pc1 in coordinate of pc2 with unique correspondence transformation======
    if debug_mode, visualize_with_corr(pc1, pc2, corrIndex, Tmat{layers},pc_bestNode_distr); end
    
        %======visualize the optical flow map, and find whether the unique_correspondences are true or not======
    if debug_mode, visualize_energy_map(pc1, pc2, corrIndex, camera_para); end
    
    %% sparse2dense_v2,interpolate sparse unique correspondence point cloud into dense point cloud======
    tic;
    warpedPointcloud = get_warped_pointcloud(pc1, pc2, corrIndex, camera_para, Tmat, pc_bestNode_distr, frame_no);
    fprintf('get_warped_pointcloud time = %d\n',toc);
    %% construct a nodeGraph======
    nodeG_thisFrame = cell(1,layers);
    
%     for i = 1:layers
%         nodeG_thisFrame{i} = {Tmat{i}',node_set_updated{i}.Location(:,:)};
%     end

    %%%%%%%%%% new %%%%%%%%%%%%%%
    tic;
    for L = 1:layers
        last_node_num = size(nodeGraph{cnt-1,2}{L}{1},1);
        new_node_num = node_set_updated{1,L}.Count;
        if new_node_num > last_node_num
           res = [nodeGraph{cnt-1,2}{L}{3};ones(new_node_num-last_node_num,1)*frame_no];
           nodeG_thisFrame{L} = {Tmat{L}',node_set_updated{L}.Location(:,:),res};
        else
            nodeG_thisFrame{L} = {Tmat{L}',node_set_updated{L}.Location(:,:),nodeGraph{cnt-1,2}{L}{3}};
        end
    end
    toc;
    %%%%%%%%%%%%%%%%%%%%%%%%
    nodeGraph(cnt,:) = {frame_no, nodeG_thisFrame};
end


% check whether each nodes can be conquered by any of the node from higher layer.
% if find, add new nodes in lower layer
function [HAVE_ADD_LOWER_LAYER_NODE, node_set_updated] = update_node_set(node_set_updated, node_r)
    L = size(node_set_updated, 2);
    HAVE_ADD_LOWER_LAYER_NODE = false;
    for l = fliplr(2:L)
        node_isolate_array = zeros(0,0);
        node_set_higher = node_set_updated{l}.Location;
        node_set_lower = node_set_updated{l-1}.Location;
        %find all the isolate node in l layer
        for i = 1:size(node_set_higher,1)
            if checkNN(node_set_lower, node_set_higher(i,:),node_r(l-1))
                node_isolate_array(end+1,:) = node_set_higher(i,:);
            end
        end
        
        if ~isempty(node_isolate_array) %add node to lower layer
            node_add = pcdownsample(pointCloud(node_isolate_array),'gridAverage',node_r(l-1)*2);
            node_add = node_add.Location;
            node_set_lower = [node_set_lower; node_add];
            node_set_updated{l-1} = pointCloud(node_set_lower);
            HAVE_ADD_LOWER_LAYER_NODE = true;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [HAVE_ADD_LOWER_LAYER_NODE, node_set_updated] = update_node_set_mean_window(node_set_updated, node_r,pc1)
    L = size(node_set_updated, 2);
    HAVE_ADD_LOWER_LAYER_NODE = false;
    for l = fliplr(2:L)
        node_isolate_array = zeros(0,0);
        node_set_higher = node_set_updated{l}.Location;
        node_set_lower = node_set_updated{l-1}.Location;
        %find all the isolate node in l layer
        for i = 1:size(node_set_higher,1)
            if checkNN(node_set_lower, node_set_higher(i,:),node_r(l-1))
                node_isolate_array(end+1,:) = node_set_higher(i,:);
            end
        end
        
        if ~isempty(node_isolate_array) %add node to lower layer
            node_add = pcdownsample(pointCloud(node_isolate_array),'gridAverage',node_r(l-1)*2);
            
            node_add = attach_new_node_to_NNP(node_add,pc1);
            
            node_add = node_add.Location;
            node_set_lower = [node_set_lower; node_add];
            node_set_updated{l-1} = pointCloud(node_set_lower);
            HAVE_ADD_LOWER_LAYER_NODE = true;
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5


%==judge distance between higher node and its nearest lower node is larger than node_radius of lower nodes or not==
function flag = checkNN(node_set_lower, node_higher, r_lower)
    flag = 0; %node_higher can be conquer by one node of node_set_lower
    n_map = repmat(node_higher, size(node_set_lower,1), 1);
    dist = sqrt(sum((node_set_lower - n_map).^2, 2));
    if min(dist)>r_lower, flag = 1; end
end


% node_added_set: pointCloud. Record position of added nodes
% L: Highest layer
function node_set_updated = add_new_node_to_highest_layer(node_set_updated, node_added_set, L)
    if ~isempty(node_added_set)
        pc_array = node_set_updated{L}.Location;
        n = size(node_added_set.Location,1);
        pc_array(end+1:end+n,:) = node_added_set.Location;
        node_set_updated{L} = pointCloud(pc_array);
    end
    
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



function node_set_updated = update_node_mean_window(nodeGraph, cnt, windowSize, frame_no)
    data = nodeGraph{cnt-1,2}; 
    layers = size(data,2);
    node_set_updated = cell(1,layers);
    weight = (1:windowSize)'/sum(1:windowSize);
    for L = 1:layers
        node_num = size(data{L}{2},1);
        Tmat = data{L}{1}; pos = data{L}{2};
        un_array = zeros(node_num,3); %updated nodes array
        for n = 1:node_num
            un = transformPointsForward(Tmat{n},pos(n,:));
            %sliding mean window
            node_start_frame = data{L}{3}(n);
            %%%%%%%%%%%%%%%%%%%%%%%% sliding %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if frame_no - node_start_frame >= windowSize - 1
                window_data = zeros(3,windowSize);
                w_s = windowSize - 1;
                while w_s > 0
                    window_data(1:3, windowSize-w_s) = nodeGraph{cnt-w_s,2}{L}{2}(n,:)';
                    w_s = w_s - 1;
                end
                window_data(1:3,windowSize) = un;
                un = window_data * weight;
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            un_array(n,:) = un;  
        end
        node_set_updated{L} = pointCloud(un_array);
    end
end



function drawNodeSeg(pc,n_r,node_set)
    [x,y,z] = sphere(15);    
    spec = {'r+','r.','ro','r*','r<','g+','g.','go','g*','g<','b+','b.','bo',...
        'b*','b<','k+','k.','ko','k*','k<','c+','c.','co','c*','c<','r+','r.','ro','r*','r<'};
    
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


%     for L = 1:length(n_r)
%         figure(L); pcshow(pc);hold on;
%         node_num = node_set{L}.Count;
%         for i = 1:node_num
%             plot3(node_set{L}.Location(i,1),node_set{L}.Location(i,2),node_set{L}.Location(i,3),spec{i},'markerSize',10)
%         end
%         xlabel('X'),ylabel('Y'),zlabel('Z'); title(['layer ',int2str(L)]);
%         hold off;
%         legend('show');
%     end
    
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
                [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPoint','Verbose',true,...
                    'InlierRatio',0.75);
                disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
            else % L=2,3,4
                if moving_pc{L}{n}.Count < 100 || fixed_pc{L}{n}.Count < 100
                    Tmat{L}{n} = Tmat{L-1}{node_tree{L}(n)};
                    rmse{L}{n} = inf;
                    disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
                    continue;
                end
                [Tmat{L}{n},~,rmse{L}{n}] = pcregrigid(moving_pc{L}{n},fixed_pc{L}{n},'Metric','pointToPoint','Verbose',true,...
                    'InitialTransform',Tmat{L-1}{node_tree{L}(n)},'InlierRatio',0.75);
                disp(['--------------now is layer ',int2str(L),' the ',int2str(n),'th node']);
            end
        end
    end
end

%
%% createNodeTree:every element in node tree record the connection with the parent node( in upper class)
function node_tree = createNodeTree(node_set,layers)
    if layers < 2
        disp('need not create node tree');
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


%% ATTACH_NEW_NODE_TO_NNP
function node_set = attach_new_node_to_NNP(node_set, pc1)
    array = zeros(node_set.Count,3);
    for n = 1:node_set.Count
        node_pos = node_set.Location(n,:);
        [indice,~] = findNearestNeighbors(pc1,node_pos,1);
        new_pos = pc1.Location(indice,:);
        array(n,:) = new_pos;
    end
    node_set = pointCloud(array);
end

%% EXCLUDE_ADDED_NODES
function node_added_set = excludeAddedNodes(node_added_set, pc1, para_set)
    dist_thres = para_set.node_radius/2;
    num_thres = para_set.OOR_thres*2;
    r = para_set.node_radius(para_set.nodeGraph_layers);
    %===node管辖的点少于阈值，则撤销===
    node_array = [];
    for i = 1:node_added_set.Count
        pos = node_added_set.Location(i,:);
        [indice, ~] = findNeighborsInRadius(pc1,pos,r);
        num = size(indice,1);
        if num > num_thres
            node_array = [node_array; pos,num];
        end
    end
    %===距离太近的node合并===
    if ~isempty(node_added_set) && ~isempty(node_array)
        node_added_set = pcdownsample(pointCloud(node_array(:,1:3)),'gridAverage',2*r);
        node_added_set = attach_new_node_to_NNP(node_added_set, pc1);
    else
        node_added_set = [];
    end
%     
%     figure(22),pcshow(pc1),hold on;
%     plot3(node_array(:,1),node_array(:,2),node_array(:,3),'ro','Markersize',10);
%     title('before dist combination');hold off
%     
%     figure(23),pcshow(pc1),hold on;
%     plot3(node_added_set.Location(:,1),node_added_set.Location(:,2),node_added_set.Location(:,3),'ro','Markersize',10);
%     title('after dist combination');hold off
end