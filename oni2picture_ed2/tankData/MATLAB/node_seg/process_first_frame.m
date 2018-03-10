%% PROCESS_FIRST_FRAME
% pc1:pointCloud. Here pc1 is in last camera frame coo
% pc2:pointCloud. back-projected from live depth map
%
% Mention: As canonical frame, pc1 of fisrt frame should be filtered in ROI to avoid unnecessary node distribution
function [warpedPointcloud, nodeGraph, DoF_node_relation_map]= process_first_frame(pc1, pc2, para_set, frame_no, cnt, DoF_node_relation_map)
    global debug_mode;
    camera_para = para_set.camera_para;
    radius_coff = para_set.radius_coff;
    pc1 = pcdenoise(pc1); pc2 = pcdenoise(pc2); 
%     roi = [-230,inf;-inf,inf;0,980]; %197-198
    roi = [-350,200,-250,100,700,900]; %frame 1 
%     roi = [-inf,inf;-inf,inf;-inf,inf]; %frame 122

    if debug_mode, figure(101),pcshow(pc1),title(sprintf('%d, before ROI filter',frame_no));  xlabel('x'),ylabel('y'),zlabel('z');end
    pc1 = select(pc1,findPointsInROI(pc1,roi)); % place wajueji in ROI
    if debug_mode, figure(100),pcshow(pc1),title(sprintf('%d, after ROI filter',frame_no));  xlabel('x'),ylabel('y'),zlabel('z');end
    
%% hierarchical node
    layers = para_set.nodeGraph_layers; 
    node_r = para_set.node_radius;
    node_set = cell(1,layers); %save position of nodes in each layer
    for L = 1:layers
        node_set{L} = pcdownsample(pc1, 'gridAverage', node_r(L)*2); % get position of nodes in each layer
    end
    
    %======attach new node to its NN point in pointcloud======
    node_set = attach_new_node_to_NNP(node_set, pc1, layers);
    %=============visualize result of node segmentation=============%
    if debug_mode, drawNodeSeg(pc1,node_r,node_set); drawNodeSeg(pc1,node_r*radius_coff,node_set); end
    
    
%% create hierarchical node tree
    node_tree = createNodeTree(node_set,layers); % connect nodes with those in higher layer corresponding 
    
    
%% find pointcloud belongs to each node
    node_r_pc1 = node_r*radius_coff; node_r_pc2 = node_r*radius_coff*1.1;
    [pc_set1, pc_set1_node_index]= distr_pc(pc1,node_r_pc1,node_set,layers,1);  %distribute pointcloud
    [pc_set2, ~]= distr_pc(pc2,node_r_pc2,node_set,layers,2);
    
    
%% hierarchical node ICP
    [Tmat, rmse]= hierarchical_ICP(pc_set1,pc_set2,layers, node_tree);
    this_file = '/home/hhg/Documents/myGithub2/tool/oni2picture_ed2/tankData/MATLAB/mat_data/';
%     if debug_mode 
%     save([this_file,'Tmat_wc0.mat'],'Tmat','-append');  
%     save([this_file,'rmse_wc0.mat'],'rmse','-append');
% %     analyseTmat(Tmat);%visualize rotation angle and translation
%     end
    
%% find best connection between node and each point in pointcloud
    thres_outlier = 2;        %mm
%     [outlier_index, pc_bestNode_distr] = findBestNode(pc1,pc2,Tmat, pc_set1_node_index, layers, thres_outlier);
    [outlier_index, pc_bestNode_distr] = findBestNode2(pc1,pc2,Tmat,pc_set1_node_index, layers, thres_outlier, node_set);
    
    corrIndex = find_unique_corr(pc_bestNode_distr, pc1, pc2, Tmat{layers}, thres_outlier);
    
%     if exist('./output/result/corr_1_200.mat','file')
%         delete('./output/result/corr_1_200.mat');
%     end
    
%     uniq_name = ['corrIndex_',int2str(frame_no),'_',int2str(frame_no+1)];
%     eval([uniq_name,'=corrIndex']);
%     save('./output/result/corr_1_200.mat',uniq_name);
    
    %======visualize pc1 in coordinate of pc2 with unique correspondence transformation======
    if debug_mode, visualize_with_corr(pc1, pc2, corrIndex, Tmat{layers},pc_bestNode_distr); end
    
    %======visualize the optical flow map, and find whether the unique_correspondences are true or not======
    if debug_mode, visualize_energy_map(pc1, pc2, corrIndex, camera_para); end
    
%% IRP_analysis
%     DoF_node_relation_map = IRP_analysis(pc1, pc2, corrIndex, 0.3, DoF_node_relation_map);
%     DoF_node_relation_map = IRP_analysis(pc1, pc2, corrIndex, para_set.OOR_thres, DoF_node_relation_map);
    
    
%% sparse2dense_v2,interpolate sparse unique correspondence point cloud into dense point cloud
    warpedPointcloud = get_warped_pointcloud(pc1, pc2, corrIndex, camera_para, Tmat, pc_bestNode_distr,frame_no);

    %======construct a nodeGraph======
    nodeG_thisFrame = cell(1,layers);
    for i = 1:layers
%         nodeG_thisFrame{i} = {Tmat{i}',node_set{i}.Location(:,:)};
        nodeG_thisFrame{i} = {Tmat{i}',node_set{i}.Location(:,:),ones(node_set{i}.Count,1)*frame_no};
    end
    nodeGraph = cell(1,2);
    nodeGraph(cnt,:) = {frame_no, nodeG_thisFrame};

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
function node_set = attach_new_node_to_NNP(node_set, pc1, layers)
    for L = 1:layers
        array = zeros(node_set{L}.Count,3);
        for n = 1:node_set{L}.Count
            node_pos = node_set{L}.Location(n,:);
            [indice,~] = findNearestNeighbors(pc1,node_pos,1);
            new_pos = pc1.Location(indice,:);
            array(n,:) = new_pos;
        end
        node_set{L} = pointCloud(array);
    end
end

