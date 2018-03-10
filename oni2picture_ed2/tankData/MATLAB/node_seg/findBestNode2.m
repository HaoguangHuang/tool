%% findBestNode2:find best connection between pointcloud and node according to result of ICP. 
%   difference between Func 'findBestNode':As for points belonging to more than one node, the best node  
%                                         for them are their nearest node instead of those have best 
%                                         transformation.
%                 It should be mention that each point can match only one node finally.

%               pc1: pointCloud. moving pointcloud in ICP
%               pc2: pointCloud. fixing pointcloud in ICP
%              Tmat: cell.       save ICP result of each node in each layers
%pc_set1_node_index: array.      n*1. record whether points in pc1 have a connection to each node in highest layer or not 
%     outlier_index: array.      n*1. record whether point in pc1 is a outlier or not
% pc_bestNode_distr: array.      n*1. record which node have best connection with point in pc1

function [outlier_index, pc_bestNode_distr] = findBestNode2(pc1, pc2, Tmat, pc_set1_node_index, layers, thres, node_set)
    if nargin < 4, error('function FINDBESTNODE don''t have enough parameters!');
    elseif nargin < 5, layers = 4;           %mm
    elseif nargin < 6, thres = 1;            %mm
    end
    
    global debug_mode;
    
    tmp = sum(pc_set1_node_index,2);
    one_index = tmp == 1;   morethanone_index = tmp > 1;
    pc_bestNode_distr = zeros(pc1.Count,1); outlier_index = zeros(pc1.Count,2);
    tt = zeros(pc1.Count,1);  dd = zeros(pc1.Count,1); dd(:,2) = ones(pc1.Count,1)*inf;
    
    tt(:,1) = linspace(1,pc1.Count,pc1.Count);  
    dd(:,1) = linspace(1,pc1.Count,pc1.Count);
    outlier_index(:,1) = linspace(1,pc1.Count,pc1.Count);
    %%========process point belonging to one node first=========
    
    for i = tt(one_index)'
       pc_bestNode_distr(i,1) = find(pc_set1_node_index(i,:));
       
       %%========judege outlier=========
%        T = Tmat{layers}{pc_bestNode_distr(i,1)}; 
%        point_afterTran = transformPointsForward(T,pc1.Location(i,:));
%        [~,dist] = findNearestNeighbors(pc2,point_afterTran,1);
%        dd(i,2) = dist;
%        if dist > thres, outlier_index(i,2) = 1; end
    end
    %%========process point belonging to more than one node first=========
    node_pc = node_set{1,layers};
    for i = tt(morethanone_index)'
        %---find NN
        pt = pc1.Location(i,:);
        n = findNearestNeighbors(node_pc,pt,1);
        pc_bestNode_distr(i,1) = n;
        %%========judege outlier=========
%         T = Tmat{layers}{n};
%         point_afterTran = transformPointsForward(T,pc1.Location(i,:)); %1*3
%         [~,dist] = findNearestNeighbors(pc2,point_afterTran,1);
%         if dist > thres, outlier_index(i,2) = 1; end
    end
    
    
    %%===========visualize relation between inlier rate and thres value==========
    if debug_mode
        count = 1;t_thres = 0:0.1:3; inliner_rate = zeros(1,size(t_thres,2));
        for t = t_thres
        inlier_rate(1,count) = sum(dd(dd(:,2)<inf,2)<t,1)/pc1.Count;     % here point controlled by none node is regarded as outlier
        count = count + 1;
        end
        figure(100),bar(t_thres,inlier_rate,0.4);xlabel('thres/mm');ylabel('inlier rate');title('ICP result of layer 4');grid on;
    end
    %%===========visualize outlier in pc1=================
    if debug_mode
        pc1_outlier = select(pc1,outlier_index(outlier_index(:,2)==1,1));% here point controlled by none node is regarded as inlier
        pc1_outlier.Color = repmat(uint8([255,0,0]),pc1_outlier.Count,1);
        figure(101);pcshow(pc1);hold on; pcshow(pc1_outlier);title(['outlier(red) in pc1, thres=',num2str(thres),'mm']);hold off;
    end
    
end